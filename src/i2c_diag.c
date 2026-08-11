#include "i2c_diag.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <nrfx.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_twim.h>

/* Pin assignments mirror the pinctrl groups in the board DTS:
 *   i2c0 (HM01B0)   SCL P0.00  SDA P0.05   -- no bias-pull-up, external only
 *   i2c1 (ICM-42686) SCL P0.28 SDA P0.02   -- bias-pull-up set in pinctrl
 */
#define I2C0_SCL_PIN 0
#define I2C0_SDA_PIN 5
#define I2C1_SCL_PIN 28
#define I2C1_SDA_PIN 2

/* The nRF52832's internal pull resistors are ~13 kohm (datasheet RPU/RPD,
 * typ).  I2C pull-ups are normally 2.2k-10k, so an external pull-up beats the
 * internal pull-down and the line still reads high -- that asymmetry is what
 * makes the three states below distinguishable.
 *
 * Settling: 13k into a few hundred pF of bus capacitance is a ~1-2 us time
 * constant, so 200 us is many time constants even for a heavily loaded bus. */
#define SETTLE_US 200

enum line_state {
	LINE_STUCK_LOW,   /* held low even against the internal pull-up */
	LINE_NO_PULLUP,   /* floats; no external pull-up strong enough to see */
	LINE_PULLED_UP,   /* external pull-up present and healthy */
};

static uint32_t read_with(uint32_t pin, nrf_gpio_pin_pull_t pull)
{
	nrf_gpio_cfg_input(pin, pull);
	k_busy_wait(SETTLE_US);
	return nrf_gpio_pin_read(pin);
}

/* Order matters: the raw read has to happen before we charge the pin with an
 * internal pull, otherwise a floating line reports whatever the last pull left
 * on it. */
static enum line_state probe_line(uint32_t pin, uint32_t *raw,
				  uint32_t *with_pu, uint32_t *with_pd)
{
	*raw     = read_with(pin, NRF_GPIO_PIN_NOPULL);
	*with_pu = read_with(pin, NRF_GPIO_PIN_PULLUP);
	*with_pd = read_with(pin, NRF_GPIO_PIN_PULLDOWN);

	if (*with_pu == 0) {
		/* 13k to VDD could not lift it: something is actively driving
		 * low -- a slave mid-transfer, a stuck device, or a short. */
		return LINE_STUCK_LOW;
	}
	if (*with_pd == 1) {
		/* Survived a 13k pull-down, so an external pull-up is winning. */
		return LINE_PULLED_UP;
	}
	return LINE_NO_PULLUP;
}

static const char *state_str(enum line_state s)
{
	switch (s) {
	case LINE_PULLED_UP: return "PULLED UP (external)";
	case LINE_NO_PULLUP: return "NO PULL-UP (floating)";
	default:             return "STUCK LOW (driven)";
	}
}

static enum line_state report_line(const char *name, uint32_t pin)
{
	uint32_t raw, pu, pd;
	enum line_state st = probe_line(pin, &raw, &pu, &pd);

	printk("    %s P0.%02u : raw=%u pu=%u pd=%u  -> %s\n",
	       name, pin, raw, pu, pd, state_str(st));
	return st;
}

static void bus_diag(const char *label, NRF_TWIM_Type *twim,
		     uint32_t scl_pin, uint32_t sda_pin)
{
	/* Take the pins back from TWIM, then hand them over exactly as found. */
	uint32_t saved_enable = twim->ENABLE;
	uint32_t saved_scl    = NRF_P0->PIN_CNF[scl_pin];
	uint32_t saved_sda    = NRF_P0->PIN_CNF[sda_pin];

	twim->ENABLE = 0;

	printk("  %s\n", label);
	enum line_state scl = report_line("SCL", scl_pin);
	enum line_state sda = report_line("SDA", sda_pin);

	if (scl == LINE_PULLED_UP && sda == LINE_PULLED_UP) {
		printk("    verdict: bus OK electrically "
		       "(a failing transfer means the device is absent or unpowered)\n");
	} else if (scl == LINE_STUCK_LOW || sda == LINE_STUCK_LOW) {
		printk("    verdict: LINE HELD LOW -- a slave is hung or a line is "
		       "shorted to GND; clock out 9 pulses to recover, or power-cycle\n");
	} else {
		printk("    verdict: NO PULL-UPS -- the bus can never idle high, so every "
		       "transfer times out; fit external resistors or set bias-pull-up\n");
	}

	NRF_P0->PIN_CNF[scl_pin] = saved_scl;
	NRF_P0->PIN_CNF[sda_pin] = saved_sda;
	twim->ENABLE = saved_enable;
}

void i2c_diag_all(void)
{
	printk("\n=== I2C line check ===\n");
	printk("  raw/pu/pd = level read with no pull / internal pull-up / "
	       "internal pull-down (~13k)\n");

	bus_diag("i2c0  HM01B0 camera", NRF_TWIM0, I2C0_SCL_PIN, I2C0_SDA_PIN);
	bus_diag("i2c1  ICM-42686 IMU", NRF_TWIM1, I2C1_SCL_PIN, I2C1_SDA_PIN);
	printk("\n");
}
