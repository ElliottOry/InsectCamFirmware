#include "HM01B0_SPI.h"
#include <nrfx_gpiote.h>
#include <errno.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/spi.h>

static nrfx_gppi_handle_t gate_ppi_ch;

/* Zephyr SPI DT spec for the SPIS node so call sites can use
 * spis_dev.bus (device) and &spis_dev.config (config) with spi_* APIs.
 */
struct spi_dt_spec spis_dev = SPI_DT_SPEC_GET(SPI_NODE, SPI_OP, 0);



static void scope_pin_init(void)
{
    /* Configure a GPIOTE task on PIN_OUT so we can scope SPIS END events */
    NRF_GPIO->PIN_CNF[PIN_OUT] =
        (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos)  |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    /* Use GPIOTE channel GPIOTE_CH as a task to toggle PIN_OUT */
    NRF_GPIOTE->CONFIG[GPIOTE_CH] =
        (GPIOTE_CONFIG_MODE_Task          << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_OUT                          << GPIOTE_CONFIG_PSEL_Pos)  |
        (GPIOTE_CONFIG_POLARITY_Toggle    << GPIOTE_CONFIG_POLARITY_Pos) |
        (GPIOTE_CONFIG_OUTINIT_Low        << GPIOTE_CONFIG_OUTINIT_Pos);

    static nrfx_gppi_handle_t ch;
    nrfx_gppi_conn_alloc(
        nrf_spis_event_address_get(NRF_SPIS2, NRF_SPIS_EVENT_ACQUIRED ),
        nrf_gpiote_task_address_get(NRF_GPIOTE, (nrf_gpiote_task_t)(NRF_GPIOTE_TASK_OUT_0 + GPIOTE_CH)),
        &ch);
    nrfx_gppi_conn_enable(ch);
}

void gate_trigger_init(void)
{
    NRF_GPIO->PIN_CNF[PIN_GATE] =
        (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
        (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
        (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos);

    /* Acquire on CSN rising (Lo->Hi) */
    NRF_GPIOTE->CONFIG[GPIOTE_CH_GATE] =
        (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_GATE                      << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    nrfx_gppi_conn_alloc(
        nrf_gpiote_event_address_get(NRF_GPIOTE, (nrf_gpiote_event_t)(NRF_GPIOTE_EVENT_IN_0 + GPIOTE_CH_GATE)),
        nrf_spis_task_address_get(NRF_SPIS2, NRF_SPIS_TASK_ACQUIRE),
        &gate_ppi_ch);
    nrfx_gppi_conn_enable(gate_ppi_ch);

}


static void release_trigger_init(void)
{
    /* Release on CSN falling (Hi->Lo) */
    static nrfx_gppi_handle_t rel_ppi_ch;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_HSYNC] =
        (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_GATE                      << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    nrfx_gppi_conn_alloc(
        nrf_gpiote_event_address_get(NRF_GPIOTE, (nrf_gpiote_event_t)(NRF_GPIOTE_EVENT_IN_0 + GPIOTE_CH_HSYNC)),
        nrf_spis_task_address_get(NRF_SPIS2, NRF_SPIS_TASK_RELEASE),
        &rel_ppi_ch);
    nrfx_gppi_conn_enable(rel_ppi_ch);
}

static void setup_GPIOTE_trigger(void)
{
    /* Release on CSN falling (Hi->Lo) */
    static nrfx_gppi_handle_t rel_ppi_ch;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_HSYNC] =
        (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_GATE                      << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    nrfx_gppi_conn_alloc(
        nrf_gpiote_event_address_get(NRF_GPIOTE, (nrf_gpiote_event_t)(NRF_GPIOTE_EVENT_IN_0 + GPIOTE_CH_HSYNC)),
        nrf_spis_task_address_get(NRF_SPIS2, NRF_SPIS_TASK_RELEASE),
        &rel_ppi_ch);
    nrfx_gppi_conn_enable(rel_ppi_ch);
}





/* Park/unpark the raw GPIOTE channels (scope pin task, SPIS ACQUIRE/RELEASE
 * gating events on LVLD/CSN).  On the nRF52832 any GPIOTE channel in Task or
 * Event mode keeps the 16 MHz clock requested through System ON sleep (~1 mA),
 * so the BLE-disconnected park must return them all to Disabled.  The PPI
 * event->task wiring stays in place: a Disabled channel neither produces
 * events nor reacts to tasks, and unpark restores the exact init configs. */
void hm_spis_gpiote_park(void)
{
    NRF_GPIOTE->CONFIG[GPIOTE_CH] = 0;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_GATE] = 0;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_HSYNC] = 0;
}

void hm_spis_gpiote_unpark(void)
{
    NRF_GPIOTE->CONFIG[GPIOTE_CH] =
        (GPIOTE_CONFIG_MODE_Task          << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_OUT                          << GPIOTE_CONFIG_PSEL_Pos)  |
        (GPIOTE_CONFIG_POLARITY_Toggle    << GPIOTE_CONFIG_POLARITY_Pos) |
        (GPIOTE_CONFIG_OUTINIT_Low        << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_GPIOTE->CONFIG[GPIOTE_CH_GATE] =
        (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_GATE                      << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[GPIOTE_CH_HSYNC] =
        (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_GATE                      << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    /* Keep the anomaly-109 interrupt guard from hm_spi_init in force. */
    NRF_GPIOTE->INTENCLR = BIT(GPIOTE_CH_GATE) | BIT(GPIOTE_CH_HSYNC);
}

void hm_spi_init(void)
{
    /* Keep the system in constant‑latency mode to avoid first‑byte corruption */
    NRF_POWER->TASKS_CONSTLAT = 1;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    scope_pin_init();
    /* Ensure SPIS ACQUIRE/RELEASE tasks are correctly gated by the LVLD/CSN
     * signal so each line transfer starts and ends cleanly.
     */
    gate_trigger_init();
    release_trigger_init();

    /* The SPIS driver's nRF52832 anomaly-109 workaround claims GPIOTE ch 7 (our
     * RELEASE channel) for a CSN falling-edge INTERRUPT at device init — before
     * pinctrl has programmed the SPIS pins, so the nrfx bookkeeping records the
     * channel as pin 0xFFFF and the GPIOTE ISR then dispatches every LVLD edge
     * (~1400/s while streaming) through an out-of-bounds handler lookup: fatal
     * or silently corrupting depending on RAM layout.  That interrupt's only
     * job is keeping the CPU awake during SPIS DMA, and CONSTLAT above already
     * guarantees that, so kill it (GATE too, defensively).  The PPI event->task
     * wiring on these channels needs no CPU and is unaffected. */
    NRF_GPIOTE->INTENCLR = BIT(GPIOTE_CH_GATE) | BIT(GPIOTE_CH_HSYNC);

    /* SPIS1 clock mode is set via SPI_OP (SPI_MODE) in HM01B0_def_values.h —
     * currently mode 0 to match the idle-low gated PCLKO. MSB first. HM01B0 drives:
     *  - SCK  on PIN_PCLK_SCK
     *  - MOSI on PIN_D0_MOSI
     *  - CSN  on PIN_LVLD_CSN (LVLD)
     * MISO is unused.
     */

}
