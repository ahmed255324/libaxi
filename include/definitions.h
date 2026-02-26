#define MM2S_CONTROL_REGISTER       0x00
#define MM2S_STATUS_REGISTER        0x04
#define MM2S_SRC_ADDRESS_REGISTER   0x18
#define MM2S_TRNSFR_LENGTH_REGISTER 0x28

#define S2MM_CONTROL_REGISTER       0x30
#define S2MM_STATUS_REGISTER        0x34
#define S2MM_DST_ADDRESS_REGISTER   0x48
#define S2MM_BUFF_LENGTH_REGISTER   0x58

#define IOC_IRQ_FLAG                1<<12
#define IDLE_FLAG                   1<<1

#define STATUS_HALTED               0x00000001
#define STATUS_IDLE                 0x00000002
#define STATUS_SG_INCLDED           0x00000008
#define STATUS_DMA_INTERNAL_ERR     0x00000010
#define STATUS_DMA_SLAVE_ERR        0x00000020
#define STATUS_DMA_DECODE_ERR       0x00000040
#define STATUS_SG_INTERNAL_ERR      0x00000100
#define STATUS_SG_SLAVE_ERR         0x00000200
#define STATUS_SG_DECODE_ERR        0x00000400
#define STATUS_IOC_IRQ              0x00001000
#define STATUS_DELAY_IRQ            0x00002000
#define STATUS_ERR_IRQ              0x00004000

#define HALT_DMA                    0x00000000
#define RUN_DMA                     0x00000001
#define RESET_DMA                   0x00000004
#define ENABLE_IOC_IRQ              0x00001000
#define ENABLE_DELAY_IRQ            0x00002000
#define ENABLE_ERR_IRQ              0x00004000
#define ENABLE_ALL_IRQ              0x00007000

#ifndef AXI_TRACE
#define AXI_TRACE 0
#endif

/* timeout per transfer in ms */
#define AXI_TIMEOUT_MS 5

/* heartbeat log interval during waits */
#define AXI_HEARTBEAT_MS 1

/* physical addresses (adjust to your system) */

#define AXI_DMA_BASE_PHYS 0x40400000u
#define DMA_MAP_SIZE 0x10000u
#define TX_BUF_PHYS 0x0E000000u
#define RX_BUF_PHYS 0x0F000000u
#define BUF_MAP_SIZE 0x10000u
#define DMA_IRQ_W1C_MASK (STATUS_IOC_IRQ | STATUS_DELAY_IRQ | STATUS_ERR_IRQ)