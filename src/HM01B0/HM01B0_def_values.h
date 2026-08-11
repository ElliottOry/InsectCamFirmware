/*
 * HM01B0Regs.h
 *
 *  Created on: July 25th, 2025
 *      Author: Elliott Ory
 */


#ifndef HM01B0DEFVALUES_H_
#define HM01B0DEFVALUES_H_
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/* SPIS clock mode.  The camera (master) drives SCK=PCLKO; the nRF SPIS samples
 * MOSI=D0.  The sampling edge MUST match the edge on which the HM01B0 presents
 * D0, or individual bits latch wrong -> "contour"/solarization corruption.
 *
 * Previously SPI_OP set neither CPOL nor CPHA, so it silently ran in MODE 0
 * despite the "mode 3" comment in hm_spi_init().  Make the mode explicit.
 *
 * Confirm the correct value with the scope (see SPI_MODE notes), then set it:
 *   - MODE 0 : (no flags)                      sample rising edge,  clk idles low
 *   - MODE 3 : SPI_MODE_CPOL | SPI_MODE_CPHA   sample rising edge,  clk idles high
 *   - MODE 1 : SPI_MODE_CPHA                    sample falling edge, clk idles low
 *   - MODE 2 : SPI_MODE_CPOL                    sample falling edge, clk idles high
 *
 * Rule: CPOL = PCLKO idle level; pick CPHA so the sampling edge lands where D0
 * is already stable (datasheet Fig 10.3 / Table 10.7: D0 transitions after the
 * PCLKO falling edge, so it is stable on the rising edge for default
 * PCLK_POLARITY 0x3068 = 0x20).
 */
/* MODE 0: the gated PCLKO idles LOW (PCLK_POLARITY 0x3068 default 0x20 = rising-edge
 * active), so CPOL must be 0 to match the idle level, and D0 is stable on the rising
 * (leading) edge -> CPHA 0.  Mode 3 (CPOL=1, idle-high) mismatched the gated clock and
 * mis-counted the first edge of each burst -> clustered black byte dropouts. */
#define SPI_MODE 0                                 /* mode 0 (matches idle-low gated PCLKO) */
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE | SPI_TRANSFER_MSB | SPI_MODE)

/* Scope/debug pin (toggled on SPIS ACQUIRED). P0.04 is bonded out on the
 * Kagafei module and otherwise unused; the old value 28 is now i2c1 SCL.
 */
#define PIN_OUT        4            /* debug pin A (ACQUIRED) */
#define GPIOTE_CH     0
/* Dedicated GPIOTE channels for scope/debug (avoid clashes with Zephyr GPIO IRQs
 * and the MCLK GPIOTE task used by the clock generator). The nRF52832 has 8 GPIOTE
 * channels (0..7). We let Zephyr's GPIO driver use the lower channels for pin
 * interrupts, the MCLK code uses task channel 5, and we keep scope/gate/release on
 * the upper channels.
 */
#define GPIOTE_CH_SCOPE_ACQ 6
#define GPIOTE_CH_SCOPE_REL 7
#define PPI_CH        0

#define IMG_WIDTH 160
#define IMG_HEIGHT 119
#define IMAGE_SIZE (IMG_WIDTH * IMG_HEIGHT)


#define PIN_MCLK 18
/* nrfx 4.0: the NRFX_*_INSTANCE macros take the peripheral base address, not
 * an instance index.  Passing a bare 3 here yields p_reg = (NRF_TIMER_Type *)3
 * and faults on first register access. */
#define MCLK_TIMER_INSTANCE NRF_TIMER3

/* Pin mapping (Kagafei ES2832AA2 module wiring) – LVLD (HSYNC) is used as
 * SPIS-CSN as well. Must match the spis2 pinctrl group in the board DTS.   */
#define PIN_VSYNC         8          /* FVLD from HM01B0                     */
#define PIN_LVLD_CSN      6          /* LVLD → CSN to SPIS2                  */
#define PIN_PCLK_SCK      7          /* PCLK → SCK to SPIS2                  */
#define PIN_D0_MOSI      20          /* D0   → MOSI to SPIS2                 */

#define CAM_FRAME_VALID                   PIN_VSYNC
#define CAM_LINE_VALID                    PIN_LVLD_CSN
//#define CAM_INT                           NRF_GPIO_PIN_MAP(0, 9)   // Input
#define CAM_MCLK_IN_FROM_MCU              PIN_MCLK


#define CAM_D0          PIN_D0_MOSI

#define CAM_SPI_CS_OUT   PIN_LVLD_CSN

#define PIN_GATE          PIN_LVLD_CSN         /* start capture when high */
#define GPIOTE_CH_GATE   4
#define PPI_CH_GATE      1

#define PIN_PCLK          PIN_PCLK_SCK
#define GPIOTE_CH_PCLK    3
#define PPI_CH_PCLK       3

/* HSYNC → timer STOP bridge (same physical LVLD line as PIN_LVLD_CSN) */
#define PIN_HSYNC         6
#define GPIOTE_CH_HSYNC   7              /* free GPIOTE channel (upper range) */
#define PPI_CH_REL        2              /* PPI channel for TASKS_STOP      */

/* Peripherals from DTS */
#define UART_NODE       DT_NODELABEL(uart0)
#define I2C_NODE        DT_ALIAS(hmm)
#define SPI_NODE        DT_NODELABEL(gendev)

/* nRF52832 RAM span that is reachable by EasyDMA (64 kB) */
#define DMA_RAM_START   0x20000000u
#define DMA_RAM_END     0x20010000u


/* FIFO item wrapper for passing image buffers through k_fifo.
 * The first field must be reserved for Zephyr's use.
 */
typedef struct frame_item {
    void *fifo_reserved;
    uint8_t *data;
    size_t len;
    /* Optional: if non-NULL, the consumer should k_sem_give() this when done
     * with the frame to let the producer know it can enqueue the next frame.
     */
    struct k_sem *consumed_sem;
    /* Optional capture-progress counter (lines fully captured so far).  Frames are
     * enqueued at frame START so transmit overlaps capture; a consumer must only
     * read data[0 .. lines_done*IMG_WIDTH - 1] and wait for more lines otherwise.
     * NULL means the frame is already complete and data[0 .. len-1] is valid.
     */
    atomic_t *lines_done;
} frame_item_t;






#endif /* HM01B0DEFVALUES_H_ */
