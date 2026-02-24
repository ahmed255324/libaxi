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

/* ---------- NEW HELPERS ---------- */

/* IOC enable bit in control register (same bit position as IOC status flag = 0x1000) */
#ifndef DMA_CTRL_IOC_EN
#define DMA_CTRL_IOC_EN  STATUS_IOC_IRQ
#endif

static int dma_wait_ioc_idle(volatile uint32_t *regs, int stat_reg, const char *who)
{
    uint32_t t0 = now_ms();
    uint32_t last_hb = t0;

    while (1) {
        uint32_t st = R(regs, stat_reg);

        if (dma_has_err(st)) {
            AXI_LOG("[AXI][ERROR] %s error while waiting IOC+IDLE\n", who);
            dma_decode_status(who, st);
            return -EIO;
        }

        /* complete when BOTH IOC and IDLE are set */
        if ((st & STATUS_IOC_IRQ) && (st & IDLE_FLAG)) {
            AXI_LOG("[AXI] %s done (IOC=1, IDLE=1)\n", who);
            dma_decode_status(who, st);
            return 0;
        }

        uint32_t t = now_ms();
        if (t - last_hb >= AXI_HEARTBEAT_MS) {
            last_hb = t;
            AXI_LOG("[AXI] %s wait... STAT=%08X (need IOC+IDLE)\n", who, st);
            dma_decode_status(who, st);
        }

        if (t - t0 > AXI_TIMEOUT_MS) {
            AXI_LOG("[AXI][ERROR] %s timeout waiting IOC+IDLE. Last STAT=%08X\n", who, st);
            dma_decode_status(who, st);
            return -ETIMEDOUT;
        }
    }
}

static void dma_stop_channel(volatile uint32_t *regs, int ctrl_reg, const char *who)
{
    (void)who;
    /* Step 2: RUN/STOP bit = 0 */
    W_LOG(regs, ctrl_reg, 0x00000000u, "STOP channel (RUN=0)");
}

static void dma_enable_ioc_channel(volatile uint32_t *regs, int ctrl_reg, const char *who)
{
    (void)who;
    /* Step 3: IOC enable (only). If you want all IRQs, use ENABLE_ALL_IRQ instead. */
    W_LOG(regs, ctrl_reg, DMA_CTRL_IOC_EN, "Enable IOC IRQ");
}

static void dma_run_channel(volatile uint32_t *regs, int ctrl_reg, const char *who)
{
    (void)who;
    /* Step 6/7: RUN=1 + IOC IRQ enable */
    W_LOG(regs, ctrl_reg, (RUN_DMA | DMA_CTRL_IOC_EN), "RUN channel + IOC IRQ");
}

/* MM2S-only transfer helper (used by axi_write for normal command TX) */
static int axi_mm2s_transfer(struct axi_dma_ctx *ctx, const uint8_t *buf, uint32_t size)
{
    if (!ctx || !ctx->regs || !ctx->tx) return -ENODEV;
    if (size == 0) return 0;
    if ((size_t)size > ctx->buf_size) return -EMSGSIZE;

    dma_dump_regs(ctx->regs, "mm2s_before");

    /* 1) Reset MM2S */
    int rc = dma_reset_channel(ctx->regs, MM2S_CONTROL_REGISTER, MM2S_STATUS_REGISTER, "MM2S");
    if (rc < 0) return rc;

    /* 2) Stop MM2S */
    dma_stop_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");

    /* 3) Enable IOC */
    dma_enable_ioc_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");

    /* clear MM2S IRQs before start */
    W_LOG(ctx->regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ");

    /* 4) Source address */
    memcpy((void*)ctx->tx, buf, (size_t)size);
    W_LOG(ctx->regs, MM2S_SRC_ADDRESS_REGISTER, ctx->tx_phys, "MM2S SRC");

    /* 6) Run MM2S */
    dma_run_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");

    /* 8) MM2S transfer length starts the transfer */
    W_LOG(ctx->regs, MM2S_TRNSFR_LENGTH_REGISTER, size, "MM2S LENGTH=start");

    dma_dump_regs(ctx->regs, "mm2s_after_start");

    /* 10) Wait IOC + IDLE */
    rc = dma_wait_ioc_idle(ctx->regs, MM2S_STATUS_REGISTER, "MM2S");
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] MM2S failed rc=%d\n", rc);
        dma_dump_regs(ctx->regs, "mm2s_failed");
        return rc;
    }

    /* optional: clear IOC for next transfer */
    W_LOG(ctx->regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ post");

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

    int rc = axi_mm2s_transfer(ctx, buf, (uint32_t)size);
    if (rc < 0)
        return rc;

    if (bytes_written)
        *bytes_written = (uint32_t)size;

    AXI_LOG("[AXI] axi_write OK wrote=%u\n", (unsigned)(bytes_written ? *bytes_written : (uint32_t)size));
    return 0;
}

static int axi_read(struct axi_driver *low, uint8_t *buf, unsigned size, uint32_t *bytes_read)
{
    if (bytes_read) *bytes_read = 0;
    if (!low || !buf) return -EINVAL;

    struct axi_dma_ctx *ctx = CTX(low);
    if (!ctx->regs || !ctx->rx || !ctx->tx) return -ENODEV;
    if ((size_t)size > ctx->buf_size) return -EMSGSIZE;

    AXI_LOG("\n[AXI] axi_read(size=%u)\n", (unsigned)size);
    dma_dump_regs(ctx->regs, "read_before");

    /* local flush command (DO NOT pass ctx->flush directly as pointer) */

    /* Optional: RX buffer clear for cleaner debug logs */
    memset((void*)ctx->rx, 0, (size_t)size);

    /* Put FLUSH byte in MM2S source buffer */
    ctx->tx = ctx->flush;

    /* ------------------------------------------------------------------
     * Follow sequence (with one practical tweak: arm S2MM length before MM2S flush length)
     * ------------------------------------------------------------------ */

    /* 1) Reset both channels */
    int rc = dma_reset_channel(ctx->regs, MM2S_CONTROL_REGISTER, MM2S_STATUS_REGISTER, "MM2S");
    if (rc < 0) return rc;
    rc = dma_reset_channel(ctx->regs, S2MM_CONTROL_REGISTER, S2MM_STATUS_REGISTER, "S2MM");
    if (rc < 0) return rc;

    /* 2) Stop both channels (RUN=0) */
    dma_stop_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");
    dma_stop_channel(ctx->regs, S2MM_CONTROL_REGISTER, "S2MM");

    /* 3) Enable IOC on both channels */
    dma_enable_ioc_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");
    dma_enable_ioc_channel(ctx->regs, S2MM_CONTROL_REGISTER, "S2MM");

    /* Clear pending IRQ flags before starting */
    W_LOG(ctx->regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ");
    W_LOG(ctx->regs, S2MM_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "S2MM W1C clear IRQ");

    /* 4) MM2S source addr -> tx buffer (flush byte) */
    W_LOG(ctx->regs, MM2S_SRC_ADDRESS_REGISTER, ctx->tx_phys, "MM2S SRC (flush)");

    /* 5) S2MM destination addr -> rx buffer */
    W_LOG(ctx->regs, S2MM_DST_ADDRESS_REGISTER, ctx->rx_phys, "S2MM DST");

    /*
     * 9) Arm S2MM FIRST (important in your design, so response is not lost)
     *    Yes, this swaps steps 8/9 intentionally for reliability with FLUSH-triggered response.
     */
    W_LOG(ctx->regs, S2MM_BUFF_LENGTH_REGISTER, (uint32_t)size, "S2MM LENGTH=start (arm receive)");

    /*
     * 8) Start MM2S by writing transfer length for FLUSH byte (1 byte)
     *    This sends command 0x09 to serializer to trigger draining the TX->RX response FIFO.
     */
    W_LOG(ctx->regs, MM2S_TRNSFR_LENGTH_REGISTER, 4, "MM2S LENGTH=start (send FLUSH)");    

    /* 7) Run S2MM */
    dma_run_channel(ctx->regs, S2MM_CONTROL_REGISTER, "S2MM");

    /* 6) Run MM2S */
    dma_run_channel(ctx->regs, MM2S_CONTROL_REGISTER, "MM2S");

    dma_dump_regs(ctx->regs, "read_after_start");

    /* 10) Wait for BOTH channels: IOC + IDLE */
    rc = dma_wait_ioc_idle(ctx->regs, MM2S_STATUS_REGISTER, "MM2S");
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] MM2S (flush) failed rc=%d\n", rc);
        dma_dump_regs(ctx->regs, "read_mm2s_failed");
        return rc;
    }

    rc = dma_wait_ioc_idle(ctx->regs, S2MM_STATUS_REGISTER, "S2MM");
    if (rc < 0) {
        AXI_LOG("[AXI][ERROR] S2MM failed rc=%d\n", rc);
        dma_dump_regs(ctx->regs, "read_s2mm_failed");
        return rc;
    }

    /* Clear IRQ flags post transfer (optional but clean) */
    W_LOG(ctx->regs, MM2S_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "MM2S W1C clear IRQ post");
    W_LOG(ctx->regs, S2MM_STATUS_REGISTER, DMA_IRQ_W1C_MASK, "S2MM W1C clear IRQ post");

    /*
     * Actual length:
     * On standard AXI DMA simple mode this register is usually the programmed length,
     * not always "actual received". If on your system it reflects actual, keep it.
     * Otherwise, fallback to requested size.
     */
    uint32_t actual = R(ctx->regs, S2MM_BUFF_LENGTH_REGISTER);
    if (actual == 0 || actual > size) {
        AXI_LOG("[AXI][WARN] S2MM_BUFF_LENGTH readback=%u, fallback to requested=%u\n",
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
