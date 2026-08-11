
#include <nrfx.h>
#include <errno.h>
  
#include "HM01B0_CLK.h"
#include "nrfx_gpiote.h"
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

nrfx_timer_t cam_timer = NRFX_TIMER_INSTANCE(MCLK_TIMER_INSTANCE);

/* File scope so hm_clk_enable() can park/unpark the GPIOTE task channel too:
 * on the nRF52832 a GPIOTE channel left in Task mode holds the 16 MHz clock on
 * through System ON sleep (~1 mA), so stopping only the timer is not enough. */
static nrfx_gpiote_t clk_gpiote_inst = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE);

static void timer_dummy_handler(nrf_timer_event_t event_type, void *p_context) {}

void hm_clk_out(void)
{

  int err_code;

  static nrfx_gppi_handle_t ppi_ch;


  err_code = nrfx_gpiote_init(&clk_gpiote_inst,
                              NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
  if ((err_code != 0) &&
      (err_code != -EALREADY)) {
    printk("GPIOTE init error %d\n", err_code);
    return;
  }

  nrfx_gpiote_output_config_t config = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
  /* Connect the input buffer so the driven level can be read back via
   * nrf_gpio_pin_read(); the GPIOTE task still owns the output.  Without this
   * the pin reads 0 whatever it is actually doing.
   */
  config.input_connect = NRF_GPIO_PIN_INPUT_CONNECT;
  nrfx_gpiote_task_config_t   task_cfg = {
      .task_ch       = 5,
      .polarity      = NRF_GPIOTE_POLARITY_TOGGLE,
      .init_val      = NRF_GPIOTE_INITIAL_VALUE_LOW,
  };
  err_code = nrfx_gpiote_output_configure(&clk_gpiote_inst,
                                          PIN_MCLK,
                                          &config,
                                          &task_cfg);
  if (err_code != 0) {
    printk("GPIOTE config error %d\n", err_code);
    return;
  }


  /* Run the timer at 16 MHz.  Output = timer_freq / (2 * CC).
   * CC = 2 -> 4 MHz MCLK.
   *
   * The datasheet floor is 3 MHz (Table 10.4 "Master Clock timing", restated in the
   * Key Parameters table and Table 6.2), so every value in the tuning history below
   * was BELOW SPEC: CC=3 (2.67 MHz) and CC=4 (2 MHz) both violate the minimum, and
   * the datasheet does not say what the part does under 3 MHz.  CC=2 (4 MHz) is the
   * slowest in-spec setting and so the closest legal point to the old "clean" ones.
   *
   * History, kept because it is about the SPIS capture path, not the sensor: 8 MHz
   * (CC=1) sat at the SPIS sampling limit (broad speckle); 4 MHz left content-
   * dependent dropouts; 2 MHz was clean.  If dropouts return at 4 MHz they must be
   * fixed on the SPIS side -- dropping MCLK below 3 MHz to hide them puts the sensor
   * out of spec.  Duty cycle must stay within 45-55% (Table 10.4); a timer/GPIOTE
   * toggle is 50% by construction.
   */
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
  timer_cfg.mode = NRF_TIMER_MODE_TIMER;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(CAM_TIMER, &timer_cfg, NULL);
  if ((err_code != 0) &&
      (err_code != -EALREADY)) {
    printk("Timer init error %d\n", err_code);
    return;
  }
  nrfx_timer_extended_compare(CAM_TIMER, NRF_TIMER_CC_CHANNEL0, 2UL,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);


  uint32_t compare_evt_addr = nrfx_timer_event_address_get(CAM_TIMER,
                                                          NRF_TIMER_EVENT_COMPARE0);
  uint32_t gpiote_task_addr = nrfx_gpiote_out_task_address_get(&clk_gpiote_inst,
                                                              PIN_MCLK);
  /* nrfx 4.0 folds channel allocation and endpoint setup into one call; the
   * channel is enabled through its handle rather than a bitmask. */
  err_code = nrfx_gppi_conn_alloc(compare_evt_addr, gpiote_task_addr, &ppi_ch);
  if (err_code != 0) {
    printk("PPI alloc error %d\n", err_code);
    return;
  }



  nrfx_gpiote_out_task_enable(&clk_gpiote_inst, PIN_MCLK);
  nrfx_gppi_conn_enable(ppi_ch);
  nrfx_timer_enable(CAM_TIMER);


  printk("HM01B0_CLK: Clock output enabled on P0.%02d\n", PIN_MCLK);
}

void hm_clk_enable(bool enable)
{
  if (enable) {
    nrfx_gpiote_out_task_enable(&clk_gpiote_inst, PIN_MCLK);
    nrfx_timer_enable(CAM_TIMER);
  } else {
    nrfx_timer_disable(CAM_TIMER);
    nrfx_gpiote_out_task_disable(&clk_gpiote_inst, PIN_MCLK);
  }
}