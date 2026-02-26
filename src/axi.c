// axi_dma_debug_more.c - Drop-in libaxi driver with VERY verbose debug
// Adds: prints ALL important #defines, prints raw register window, logs every MMIO write, sanity reads
//
// Toggle logs:
//   #define AXI_TRACE 1  -> on
//   #define AXI_TRACE 0  -> off

#include <float.h>
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/time.h>

#include "../include/axi.h"
#include "../include/definitions.h"

/* ============================== LOGGING ============================== */
#if AXI_TRACE
static void AXI_LOG(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
}
static void dump_hex_limited(const char *tag, const uint8_t *buf, size_t n)
{
    size_t show = (n > 1024) ? 1024 : n;
    AXI_LOG("[AXI] %s (%zu bytes, show %zu): ", tag, n, show);
    for (size_t i = 0; i < show; i++) {
        AXI_LOG("%02X", buf[i]);
        if ((i % 4) == 3) AXI_LOG(" ");
        else AXI_LOG(" ");
    }
    if (show < n) AXI_LOG("...");
    AXI_LOG("\n");
}
#else
static void AXI_LOG(const char *fmt, ...) { (void)fmt; }
static void dump_hex_limited(const char *tag, const uint8_t *buf, size_t n)
{ (void)tag; (void)buf; (void)n; }
#endif

static inline uint32_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000u + tv.tv_usec / 1000u);
}

/* ============================== PRIVATE CONTEXT ============================== */
/* IMPORTANT: pub must be first */
struct axi_dma_ctx {
    struct axi_driver pub;

    int mem_fd;
    volatile uint32_t *regs;
    volatile uint8_t  *tx;
    volatile uint8_t  *rx;

    uint32_t dma_base;
    uint32_t tx_phys;
    uint32_t rx_phys;
    size_t   buf_size;
    uint8_t flush[4];
};

static inline struct axi_dma_ctx *CTX(struct axi_driver *low)
{
    return (struct axi_dma_ctx*)low;
}

static inline uint32_t R(volatile uint32_t *regs, int off)
{
    return regs[off >> 2];
}

/* log every MMIO write with offset+value */
static inline void W_LOG(volatile uint32_t *regs, int off, uint32_t v, const char *why)
{
#if AXI_TRACE
    AXI_LOG("[AXI][MMIO] W32 off=0x%02X val=0x%08X  // %s\n", off, v, why ? why : "");
#endif
    regs[off >> 2] = v;
}

static int dma_has_err(uint32_t st)
{
    return (st & (STATUS_DMA_INTERNAL_ERR | STATUS_DMA_SLAVE_ERR | STATUS_DMA_DECODE_ERR | STATUS_ERR_IRQ)) != 0;
}

static void dma_decode_status(const char *who, uint32_t st)
{
#if AXI_TRACE
    AXI_LOG("[AXI] %s STAT=%08X  HALTED=%d IDLE=%d IOC=%d DLY=%d ERR_IRQ=%d  INT=%d SLV=%d DEC=%d\n",
        who, st,
        !!(st & STATUS_HALTED),
        !!(st & STATUS_IDLE),
        !!(st & STATUS_IOC_IRQ),
        !!(st & STATUS_DELAY_IRQ),
        !!(st & STATUS_ERR_IRQ),
        !!(st & STATUS_DMA_INTERNAL_ERR),
        !!(st & STATUS_DMA_SLAVE_ERR),
        !!(st & STATUS_DMA_DECODE_ERR));
#else
    (void)who; (void)st;
#endif
}

/* dump a raw window of registers: [start..end] step 4 */
static void dma_dump_raw(volatile uint32_t *regs, const char *tag, int start, int end)
{
#if AXI_TRACE
    AXI_LOG("\n[AXI][%s] RAW reg dump 0x%02X..0x%02X:\n", tag, start, end);
    for (int off = start; off <= end; off += 4) {
        uint32_t v = R(regs, off);
        AXI_LOG("  +0x%02X : 0x%08X\n", off, v);
    }
#endif
}

static void dma_dump_regs(volatile uint32_t *regs, const char *tag)
{
#if AXI_TRACE
    uint32_t mm2s_ctrl = R(regs, MM2S_CONTROL_REGISTER);
    uint32_t mm2s_stat = R(regs, MM2S_STATUS_REGISTER);
    uint32_t mm2s_src  = R(regs, MM2S_SRC_ADDRESS_REGISTER);
    uint32_t mm2s_len  = R(regs, MM2S_TRNSFR_LENGTH_REGISTER);

    uint32_t s2mm_ctrl = R(regs, S2MM_CONTROL_REGISTER);
    uint32_t s2mm_stat = R(regs, S2MM_STATUS_REGISTER);
    uint32_t s2mm_dst  = R(regs, S2MM_DST_ADDRESS_REGISTER);
    uint32_t s2mm_len  = R(regs, S2MM_BUFF_LENGTH_REGISTER);

    AXI_LOG("\n[AXI][%s] DMA regs:\n", tag);
    AXI_LOG("  MM2S: CTRL=%08X STAT=%08X SRC=%08X LEN=%08X\n", mm2s_ctrl, mm2s_stat, mm2s_src, mm2s_len);
    dma_decode_status("MM2S", mm2s_stat);
    AXI_LOG("  S2MM: CTRL=%08X STAT=%08X DST=%08X LEN=%08X\n", s2mm_ctrl, s2mm_stat, s2mm_dst, s2mm_len);
    dma_decode_status("S2MM", s2mm_stat);

    /* extra: raw dump of full common range */
    //dma_dump_raw(regs, tag, 0x00, 0x5C);
#else
    (void)regs; (void)tag;
#endif
}

static void dump_definitions(void)
{
#if AXI_TRACE
    AXI_LOG("\n[AXI] ====== BUILD/DEFINE DUMP ======\n");
    AXI_LOG("[AXI] AXI_DMA_BASE_PHYS=0x%08X DMA_MAP_SIZE=0x%X\n", (unsigned)AXI_DMA_BASE_PHYS, (unsigned)DMA_MAP_SIZE);
    AXI_LOG("[AXI] TX_BUF_PHYS=0x%08X RX_BUF_PHYS=0x%08X BUF_MAP_SIZE=0x%X\n",
            (unsigned)TX_BUF_PHYS, (unsigned)RX_BUF_PHYS, (unsigned)BUF_MAP_SIZE);
    AXI_LOG("[AXI] TIMEOUT_MS=%u HEARTBEAT_MS=%u\n", (unsigned)AXI_TIMEOUT_MS, (unsigned)AXI_HEARTBEAT_MS);

    AXI_LOG("[AXI] Offsets:\n");
    AXI_LOG("  MM2S_CONTROL_REGISTER       =0x%02X\n", MM2S_CONTROL_REGISTER);
    AXI_LOG("  MM2S_STATUS_REGISTER        =0x%02X\n", MM2S_STATUS_REGISTER);
    AXI_LOG("  MM2S_SRC_ADDRESS_REGISTER   =0x%02X\n", MM2S_SRC_ADDRESS_REGISTER);
    AXI_LOG("  MM2S_TRNSFR_LENGTH_REGISTER =0x%02X\n", MM2S_TRNSFR_LENGTH_REGISTER);
    AXI_LOG("  S2MM_CONTROL_REGISTER       =0x%02X\n", S2MM_CONTROL_REGISTER);
    AXI_LOG("  S2MM_STATUS_REGISTER        =0x%02X\n", S2MM_STATUS_REGISTER);
    AXI_LOG("  S2MM_DST_ADDRESS_REGISTER   =0x%02X\n", S2MM_DST_ADDRESS_REGISTER);
    AXI_LOG("  S2MM_BUFF_LENGTH_REGISTER   =0x%02X\n", S2MM_BUFF_LENGTH_REGISTER);

    AXI_LOG("[AXI] Control bits:\n");
    AXI_LOG("  RUN_DMA=0x%08X RESET_DMA=0x%08X ENABLE_ALL_IRQ=0x%08X\n",
            (unsigned)RUN_DMA, (unsigned)RESET_DMA, (unsigned)ENABLE_ALL_IRQ);

    AXI_LOG("[AXI] Status bits:\n");
    AXI_LOG("  STATUS_HALTED=0x%08X STATUS_IDLE=0x%08X IDLE_FLAG=0x%08X\n",
            (unsigned)STATUS_HALTED, (unsigned)STATUS_IDLE, (unsigned)IDLE_FLAG);
    AXI_LOG("  IOC=0x%08X DLY=0x%08X ERR_IRQ=0x%08X\n",
            (unsigned)STATUS_IOC_IRQ, (unsigned)STATUS_DELAY_IRQ, (unsigned)STATUS_ERR_IRQ);
    AXI_LOG("  INT_ERR=0x%08X SLV_ERR=0x%08X DEC_ERR=0x%08X\n",
            (unsigned)STATUS_DMA_INTERNAL_ERR, (unsigned)STATUS_DMA_SLAVE_ERR, (unsigned)STATUS_DMA_DECODE_ERR);

    AXI_LOG("[AXI] DMA_IRQ_W1C_MASK=0x%08X\n", (unsigned)DMA_IRQ_W1C_MASK);
    AXI_LOG("[AXI] ====== END DEFINE DUMP ======\n\n");
#endif
}

static void dma_clear_irqs(volatile uint32_t *regs)
{
    W_LOG(regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ");
    W_LOG(regs, S2MM_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "S2MM W1C clear IRQ");
}

/* Proper reset: write RESET, wait until RESET clears */
static int dma_reset_channel(volatile uint32_t *regs, int ctrl_reg, int stat_reg, const char *who)
{
    AXI_LOG("[AXI] %s reset: write RESET\n", who);
    W_LOG(regs, ctrl_reg, RESET_DMA, "assert RESET");

    uint32_t t0 = now_ms();
    uint32_t last_hb = t0;

    while (1) {
        uint32_t ctrl = R(regs, ctrl_reg);
        uint32_t st   = R(regs, stat_reg);

        if ((ctrl & RESET_DMA) == 0) {
            AXI_LOG("[AXI] %s reset done (CTRL=%08X)\n", who, ctrl);
            return 0;
        }

        uint32_t t = now_ms();
        if (t - last_hb >= AXI_HEARTBEAT_MS) {
            last_hb = t;
            AXI_LOG("[AXI] %s reset wait... CTRL=%08X STAT=%08X\n", who, ctrl, st);
            dma_decode_status(who, st);
        }

        if (t - t0 > AXI_TIMEOUT_MS) {
            AXI_LOG("[AXI][ERROR] %s reset timeout CTRL=%08X STAT=%08X\n", who, ctrl, st);
            return -ETIMEDOUT;
        }
    }
}

static int dma_wait_idle(volatile uint32_t *regs, int stat_reg, const char *who)
{
    uint32_t t0 = now_ms();
    uint32_t last_hb = t0;

    while (1) {
        uint32_t st = R(regs, stat_reg);

        if (dma_has_err(st)) {
            AXI_LOG("[AXI][ERROR] %s error while waiting\n", who);
            dma_decode_status(who, st);
            return -EIO;
        }

        if (st & IDLE_FLAG) {
            AXI_LOG("[AXI] %s done (IDLE=1)\n", who);
            dma_decode_status(who, st);
            return 0;
        }

        uint32_t t = now_ms();
        if (t - last_hb >= AXI_HEARTBEAT_MS) {
            last_hb = t;
            AXI_LOG("[AXI] %s wait... STAT=%08X\n", who, st);
            dma_decode_status(who, st);
        }

        if (t - t0 > AXI_TIMEOUT_MS) {
            AXI_LOG("[AXI][ERROR] %s timeout waiting for IDLE. Last STAT=%08X\n", who, st);
            dma_decode_status(who, st);
            return -ETIMEDOUT;
        }
    }
}

static void dma_ensure_run(volatile uint32_t *regs, int ctrl_reg, int stat_reg, const char *who)
{
    uint32_t ctrl = R(regs, ctrl_reg);
    uint32_t st   = R(regs, stat_reg);

    AXI_LOG("[AXI] ensure_run %s: CTRL=%08X STAT=%08X\n", who, ctrl, st);
    dma_decode_status(who, st);

    if ((ctrl & RUN_DMA) == 0) {
        AXI_LOG("[AXI] %s RUN=0 -> set RUN\n", who);
        W_LOG(regs, ctrl_reg, (RUN_DMA | ENABLE_ALL_IRQ), "set RUN+IRQs");
    } else if (st & STATUS_HALTED) {
        AXI_LOG("[AXI] %s HALTED=1 -> set RUN\n", who);
        W_LOG(regs, ctrl_reg, (RUN_DMA | ENABLE_ALL_IRQ), "set RUN+IRQs (was halted)");
    } else {
        /* already running */
    }
}

/* ============================== DRIVER API ============================== */
static int axi_init(struct axi_driver *low)
{
    if (!low) return -EINVAL;
    struct axi_dma_ctx *ctx = CTX(low);

    AXI_LOG("\n[AXI] axi_init() called\n");
    dump_definitions();

    ctx->dma_base = AXI_DMA_BASE_PHYS;
    ctx->tx_phys  = TX_BUF_PHYS;
    ctx->rx_phys  = RX_BUF_PHYS;
    ctx->buf_size = BUF_MAP_SIZE;

    AXI_LOG("[AXI] open /dev/mem ...\n");
    ctx->mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (ctx->mem_fd < 0) {
        AXI_LOG("[AXI][ERROR] open(/dev/mem): %s\n", strerror(errno));
        return -errno;
    }

    AXI_LOG("[AXI] mmap regs @0x%08X size=0x%X\n", ctx->dma_base, (unsigned)DMA_MAP_SIZE);
    ctx->regs = (volatile uint32_t*)mmap(NULL, DMA_MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, ctx->mem_fd, ctx->dma_base);
    if (ctx->regs == MAP_FAILED) {
        AXI_LOG("[AXI][ERROR] mmap(regs): %s\n", strerror(errno));
        close(ctx->mem_fd);
        ctx->mem_fd = -1;
        ctx->regs = NULL;
        return -errno;
    }

    AXI_LOG("[AXI] mmap tx @0x%08X size=0x%zX\n", ctx->tx_phys, ctx->buf_size);
    ctx->tx = (volatile uint8_t*)mmap(NULL, ctx->buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, ctx->mem_fd, ctx->tx_phys);
    if (ctx->tx == MAP_FAILED) {
        AXI_LOG("[AXI][ERROR] mmap(tx): %s\n", strerror(errno));
        munmap((void*)ctx->regs, DMA_MAP_SIZE);
        close(ctx->mem_fd);
        ctx->mem_fd = -1;
        ctx->regs = NULL;
        ctx->tx = NULL;
        return -errno;
    }

    AXI_LOG("[AXI] mmap rx @0x%08X size=0x%zX\n", ctx->rx_phys, ctx->buf_size);
    ctx->rx = (volatile uint8_t*)mmap(NULL, ctx->buf_size, PROT_READ|PROT_WRITE, MAP_SHARED, ctx->mem_fd, ctx->rx_phys);
    if (ctx->rx == MAP_FAILED) {
        AXI_LOG("[AXI][ERROR] mmap(rx): %s\n", strerror(errno));
        munmap((void*)ctx->tx, ctx->buf_size);
        munmap((void*)ctx->regs, DMA_MAP_SIZE);
        close(ctx->mem_fd);
        ctx->mem_fd = -1;
        ctx->regs = NULL;
        ctx->tx = NULL;
        ctx->rx = NULL;
        return -errno;
    }

    AXI_LOG("[AXI] mapped regs=%p tx=%p rx=%p\n", (void*)ctx->regs, (void*)ctx->tx, (void*)ctx->rx);

    dma_dump_regs(ctx->regs, "after_mmap_initial");

    /* reset both */
    (void)dma_reset_channel(ctx->regs, MM2S_CONTROL_REGISTER, MM2S_STATUS_REGISTER, "MM2S");
    (void)dma_reset_channel(ctx->regs, S2MM_CONTROL_REGISTER, S2MM_STATUS_REGISTER, "S2MM");

    AXI_LOG("[AXI] clear IRQs\n");
    dma_clear_irqs(ctx->regs);

    AXI_LOG("[AXI] program addresses\n");
    W_LOG(ctx->regs, MM2S_SRC_ADDRESS_REGISTER, ctx->tx_phys, "set MM2S SRC");
    W_LOG(ctx->regs, S2MM_DST_ADDRESS_REGISTER, ctx->rx_phys, "set S2MM DST");

    AXI_LOG("[AXI] set RUN+IRQs\n");
    W_LOG(ctx->regs, MM2S_CONTROL_REGISTER, (RUN_DMA | ENABLE_ALL_IRQ), "MM2S RUN");
    W_LOG(ctx->regs, S2MM_CONTROL_REGISTER, (RUN_DMA | ENABLE_ALL_IRQ), "S2MM RUN");

    /* sanity: read back CTRL regs */
    AXI_LOG("[AXI] sanity readback: MM2S_CTRL=%08X S2MM_CTRL=%08X\n",
            R(ctx->regs, MM2S_CONTROL_REGISTER),
            R(ctx->regs, S2MM_CONTROL_REGISTER));

    dma_dump_regs(ctx->regs, "after_init");
    AXI_LOG("[AXI] axi_init() done\n\n");

    return 0;
}

static int axi_quit(struct axi_driver *low)
{
    if (!low) return -EINVAL;
    struct axi_dma_ctx *ctx = CTX(low);

    AXI_LOG("\n[AXI] axi_quit() called\n");
    if (ctx->regs) {
        dma_dump_regs(ctx->regs, "before_unmap");
    }

    if (ctx->rx && ctx->rx != (void*)MAP_FAILED) munmap((void*)ctx->rx, ctx->buf_size);
    if (ctx->tx && ctx->tx != (void*)MAP_FAILED) munmap((void*)ctx->tx, ctx->buf_size);
    if (ctx->regs && ctx->regs != (void*)MAP_FAILED) munmap((void*)ctx->regs, DMA_MAP_SIZE);
    ctx->rx = NULL; ctx->tx = NULL; ctx->regs = NULL;

    if (ctx->mem_fd >= 0) close(ctx->mem_fd);
    ctx->mem_fd = -1;

    AXI_LOG("[AXI] axi_quit() done\n\n");
    return 0;
}

static int axi_write(struct axi_driver *low, uint8_t *buf, int size, uint32_t *bytes_written)
{
    if (bytes_written) *bytes_written = 0;
    if (!low || !buf || size < 0) return -EINVAL;

    struct axi_dma_ctx *ctx = CTX(low);
    if (!ctx->regs || !ctx->tx) return -ENODEV;
    if ((size_t)size > ctx->buf_size) return -EMSGSIZE;

    AXI_LOG("\n[AXI] axi_write(size=%d)\n", size);
    dump_hex_limited("TX payload", buf, (size_t)size);
    dma_dump_regs(ctx->regs, "mm2s_before");

    W_LOG(ctx->regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ");
    memcpy((void*)ctx->tx, buf, (size_t)size);

    W_LOG(ctx->regs, MM2S_SRC_ADDRESS_REGISTER, ctx->tx_phys, "MM2S SRC");
    W_LOG(ctx->regs, MM2S_TRNSFR_LENGTH_REGISTER, (uint32_t)size, "MM2S LENGTH=start");
    dma_dump_regs(ctx->regs, "mm2s_after_start");

    dma_ensure_run(ctx->regs, MM2S_CONTROL_REGISTER, MM2S_STATUS_REGISTER, "MM2S");

    int rc = dma_wait_idle(ctx->regs, MM2S_STATUS_REGISTER, "MM2S");
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] MM2S failed rc=%d\n", rc);
        dma_dump_regs(ctx->regs, "mm2s_failed");
        return rc;
    }

    if (bytes_written) *bytes_written = (uint32_t)size;
    AXI_LOG("[AXI] axi_write OK wrote=%u\n", (unsigned)*bytes_written);
    return 0;
}

static int axi_read(struct axi_driver *low, uint8_t *buf, unsigned size, uint32_t *bytes_read)
{
    if (bytes_read) *bytes_read = 0;
    if (!low || !buf) return -EINVAL;

    struct axi_dma_ctx *ctx = CTX(low);
    if (!ctx->regs || !ctx->rx) return -ENODEV;
    if ((size_t)size > ctx->buf_size) return -EMSGSIZE;

    AXI_LOG("\n[AXI] axi_read(size=%u)\n", (unsigned)size);
    dma_dump_regs(ctx->regs, "s2mm_before");

    /* IRQs clear */
    W_LOG(ctx->regs, S2MM_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "S2MM W1C clear IRQ");

    /* optional: RX buffer leeren für Debug (hilft gegen alte Daten im Log) */
    memset((void *)ctx->rx, 0, (size_t)size);

    /* S2MM armed */
    W_LOG(ctx->regs, S2MM_DST_ADDRESS_REGISTER, ctx->rx_phys, "S2MM DST");
    W_LOG(ctx->regs, S2MM_BUFF_LENGTH_REGISTER, (uint32_t)size, "S2MM LENGTH=start");
    dma_dump_regs(ctx->regs, "s2mm_after_start");

    dma_ensure_run(ctx->regs, S2MM_CONTROL_REGISTER, S2MM_STATUS_REGISTER, "S2MM");

    AXI_LOG("----------------------------flushstart--------------------");
    int rc;
    /* 2) Jetzt erst FLUSH senden (MM2S) */
    uint32_t bw = 0;
    rc = axi_write(low, ctx->flush, 4, &bw);
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] flush write failed rc=%d\n", rc);
        return rc;
    }
    if (bw != 4) {
        AXI_LOG("[AXI][ERROR] flush write short bw=%u\n", (unsigned)bw);
        return -EIO;
    }
    AXI_LOG("----------------------------flushends---------------------");

    /* 3) Auf S2MM Ende warten */
    rc = dma_wait_idle(ctx->regs, S2MM_STATUS_REGISTER, "S2MM");
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] S2MM failed rc=%d\n", rc);
        dma_dump_regs(ctx->regs, "s2mm_failed");
        return rc;
    }

    /* 4) Tatsächliche empfangene Länge lesen (bei dir zeigt das Register realen Wert) */
    uint32_t actual = R(ctx->regs, S2MM_BUFF_LENGTH_REGISTER);

    /* Schutz */
    if (actual > size) {
        AXI_LOG("[AXI][WARN] S2MM actual=%u > requested=%u, clamp\n",
                (unsigned)actual, (unsigned)size);
        actual = size;
    }

    memcpy(buf, (void*)ctx->rx, (size_t)actual);
    dump_hex_limited("RX payload", buf, (size_t)actual);

    if (bytes_read) *bytes_read = actual;
    AXI_LOG("[AXI] axi_read OK read=%u (requested=%u)\n",
            (unsigned)actual, (unsigned)size);
    return 0;
}

/* ============================== REGISTRATION ============================== */

static struct axi_dma_ctx g_ctx = {
    .pub = {
        .open  = axi_init,
        .close = axi_quit,
        .write = axi_write,
        .read  = axi_read,
        .speed = NULL
    },
    .mem_fd  = -1,
    .regs    = NULL,
    .tx      = NULL,
    .rx      = NULL,
    .dma_base= AXI_DMA_BASE_PHYS,
    .tx_phys = TX_BUF_PHYS,
    .rx_phys = RX_BUF_PHYS,
    .buf_size= BUF_MAP_SIZE,
    .flush = {0x9, 0x9, 0x9, 0x9}
};

struct axi_driver *axi_driver_register(void)
{
    AXI_LOG("[AXI] axi_driver_register(): returning driver @%p (AXI_TRACE=%d)\n",
            (void*)&g_ctx.pub, AXI_TRACE);
    return &g_ctx.pub;
}
