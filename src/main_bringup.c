#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>

#include "HM01B0_CLK.h"
#include "icm42686.h"

#define HM01B0_ADDR 0x24

/* Camera bus: i2c0 / TWIM0. */
#define PIN_SCL 0
#define PIN_SDA 5

/* IMU bus: i2c1 / TWIM1. A separate peripheral instance on separate pins, so
 * nothing here can disturb the camera's bus.
 */
#define PIN_IMU_SCL 28
#define PIN_IMU_SDA 2

/* HM01B0 uses 16-bit register addresses with 8-bit data (datasheet §7). */
#define REG_MODEL_ID_H  0x0000
#define REG_MODEL_ID_L  0x0001
#define REG_SILICON_REV 0x0002
#define REG_FRAME_COUNT 0x0005
#define REG_PIXEL_ORDER 0x0006
#define REG_MODE_SELECT 0x0100

#define HM01B0_MODEL_ID 0x01B0

/* How long to let MCLK run before touching the sensor over I2C. At the 2.67 MHz
 * MCLK we generate, this is several hundred thousand clocks -- far more than the
 * sensor's internal reset needs.
 */
#define HM01B0_MCLK_SETTLE_MS 100

static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *const imu_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

static int hm01b0_read_reg(uint16_t reg, uint8_t *val)
{
	uint8_t addr_buf[2] = { reg >> 8, reg & 0xFF };

	return i2c_write_read(i2c_dev, HM01B0_ADDR,
			      addr_buf, sizeof(addr_buf), val, 1);
}

/* The board was seen rebooting every few seconds with no fault banner, which a
 * caught CPU fault would have printed.  RESETREAS says why: a brownout or
 * power-on reset leaves every bit clear, so an all-zero value here on a repeating
 * reset means the supply is sagging -- which would also keep the sensor's rails
 * from ever coming up.  Cleared after reading, since the bits are cumulative.
 */
static void report_reset_reason(void)
{
	uint32_t reas = nrf_power_resetreas_get(NRF_POWER);

	nrf_power_resetreas_clear(NRF_POWER, reas);

	printk("Reset reason  : 0x%08x", reas);

	if (reas == 0) {
		printk(" -- power-on / BROWNOUT (no other source flagged)");
	}
	if (reas & NRF_POWER_RESETREAS_RESETPIN_MASK) {
		printk(" RESETPIN");
	}
	if (reas & NRF_POWER_RESETREAS_DOG_MASK) {
		printk(" WATCHDOG");
	}
	if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
		printk(" SOFT_RESET");
	}
	if (reas & NRF_POWER_RESETREAS_LOCKUP_MASK) {
		printk(" CPU_LOCKUP");
	}
	if (reas & NRF_POWER_RESETREAS_OFF_MASK) {
		printk(" WAKE_FROM_OFF");
	}
	if (reas & NRF_POWER_RESETREAS_DIF_MASK) {
		printk(" DEBUG_IF");
	}

	printk("\n");
}

/* The firmware previously *claimed* MCLK was running purely because hm_clk_out()
 * returned without error.  Sample the pin instead: at 2.67 MHz a tight read loop
 * sees both levels if the timer/PPI/GPIOTE chain is really toggling, and a
 * constant level if it is not.
 */
static bool mclk_is_toggling(void)
{
	unsigned int high = 0, low = 0;

	for (int i = 0; i < 1000; i++) {
		if (nrf_gpio_pin_read(PIN_MCLK)) {
			high++;
		} else {
			low++;
		}
	}

	/* This loop samples far slower than MCLK and is not phase-locked to it, so the
	 * high/low split is aliased and says nothing reliable about duty cycle.  Treat it
	 * strictly as edges-present vs DC-stuck; use a scope to judge the waveform.
	 */
	printk("MCLK (P0.%02d)  : %u high / %u low over 1000 aliased samples -- %s\n",
	       PIN_MCLK, high, low,
	       (high && low) ? "edges present (duty NOT measurable here)" : "DC-STUCK");

	return high && low;
}

/* TWIM leaves its pins with the input buffer connected, so the idle bus level can
 * be read straight off the pads.  With pull-ups fitted, both lines must sit high
 * when nobody is driving them.  A line stuck low means something is clamping it.
 */
static void bus_idle_levels(uint8_t scl_pin, uint8_t sda_pin)
{
	int scl = nrf_gpio_pin_read(scl_pin);
	int sda = nrf_gpio_pin_read(sda_pin);

	printk("Bus idle      : SCL(P0.%02d)=%d  SDA(P0.%02d)=%d -- %s\n",
	       scl_pin, scl, sda_pin, sda,
	       (scl && sda) ? "both HIGH (pull-ups OK, bus free)" : "LINE HELD LOW");
}

static void bus_scan(const struct device *dev, uint8_t scl_pin, uint8_t sda_pin)
{
	int found = 0;

	printk("\nScanning %s (SCL=P0.%02d, SDA=P0.%02d)...\n",
	       dev->name, scl_pin, sda_pin);

	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t dummy;

		/* nRF TWIM cannot issue a zero-length write, so probe each
		 * address with a 1-byte read instead.
		 */
		if (i2c_read(dev, &dummy, 1, addr) == 0) {
			printk("  device found at 0x%02x\n", addr);
			found++;
		}
	}

	printk("Scan complete: %d device(s) responded.\n\n", found);
}

static int hm01b0_check(void)
{
	uint8_t id_h, id_l;
	uint16_t model_id;
	int err;

	static const struct {
		uint16_t reg;
		const char *name;
	} status_regs[] = {
		{ REG_SILICON_REV, "SILICON_REV" },
		{ REG_MODE_SELECT, "MODE_SELECT" },
		{ REG_FRAME_COUNT, "FRAME_COUNT" },
		{ REG_PIXEL_ORDER, "PIXEL_ORDER" },
	};

	printk("\n=== HM01B0 camera status check (i2c0) ===\n");

	/* The sensor clocks its register block from MCLK, so it will not ACK on I2C
	 * until the clock is running AND its internal power-on reset has run to
	 * completion -- and that reset is itself clocked from MCLK, so the wait only
	 * starts counting once the clock does. Probing too early looks exactly like a
	 * dead sensor: a clean NAK from a part that is simply not listening yet.
	 *
	 * HM01B0_MCLK_SETTLE_MS is deliberately generous rather than tuned to the
	 * datasheet minimum -- this is a one-shot bring-up check, so there is nothing
	 * to be gained by shaving it, and a too-short wait would hand us a false
	 * negative that reads like a hardware fault.
	 */
	hm_clk_out();
	k_msleep(HM01B0_MCLK_SETTLE_MS);

	if (!mclk_is_toggling()) {
		printk("      MCLK is not running -- the sensor cannot ACK on I2C without it.\n"
		       "      Fault is on the MCU side (timer/PPI/GPIOTE), not the camera.\n");
	}

	if (!device_is_ready(i2c_dev)) {
		printk("ERROR: %s is not ready\n", i2c_dev->name);
		return -ENODEV;
	}

	bus_idle_levels(PIN_SCL, PIN_SDA);

	err = i2c_configure(i2c_dev,
			    I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER);
	if (err) {
		printk("ERROR: i2c_configure failed (%d)\n", err);
		return err;
	}

	bus_scan(i2c_dev, PIN_SCL, PIN_SDA);

	err = hm01b0_read_reg(REG_MODEL_ID_H, &id_h);
	if (err) {
		printk("FAIL: no ACK from 0x%02x reading MODEL_ID (err %d)\n",
		       HM01B0_ADDR, err);
		printk("      The HM01B0 register block is clocked from MCLK; with\n");
		printk("      no MCLK driven the sensor will not ACK even if SDA/SCL\n");
		printk("      are wired correctly. Also check the AVDD/IOVDD/DVDD rails.\n");
		return err;
	}

	err = hm01b0_read_reg(REG_MODEL_ID_L, &id_l);
	if (err) {
		printk("FAIL: MODEL_ID_L read failed (err %d)\n", err);
		return err;
	}

	model_id = ((uint16_t)id_h << 8) | id_l;

	printk("MODEL_ID   : 0x%04x (expected 0x%04x) -- %s\n",
	       model_id, HM01B0_MODEL_ID,
	       model_id == HM01B0_MODEL_ID ? "OK" : "MISMATCH");

	for (size_t i = 0; i < ARRAY_SIZE(status_regs); i++) {
		uint8_t val;

		err = hm01b0_read_reg(status_regs[i].reg, &val);
		if (err) {
			printk("%-11s: read failed (err %d)\n",
			       status_regs[i].name, err);
			continue;
		}

		printk("%-11s: 0x%02x\n", status_regs[i].name, val);
	}

	return 0;
}

/* Integer square root, so the gravity check below needs neither libm nor
 * floating-point printf support.
 */
static uint32_t isqrt_u64(uint64_t value)
{
	uint64_t x = value;
	uint64_t y = (x + 1) / 2;

	if (value == 0) {
		return 0;
	}

	while (y < x) {
		x = y;
		y = (x + value / x) / 2;
	}

	return (uint32_t)x;
}

static int icm42686_check(void)
{
	int16_t temp_raw, accel[3], gyro[3];
	int16_t first_accel[3] = { 0 };
	uint8_t who_am_i = 0;
	bool moved = false;
	uint32_t mag_mg = 0;
	int err;

	printk("\n=== ICM-42686-P IMU status check (i2c1) ===\n");

	if (!device_is_ready(imu_i2c_dev)) {
		printk("ERROR: %s is not ready\n", imu_i2c_dev->name);
		return -ENODEV;
	}

	bus_idle_levels(PIN_IMU_SCL, PIN_IMU_SDA);
	bus_scan(imu_i2c_dev, PIN_IMU_SCL, PIN_IMU_SDA);

	err = icm42686_read_who_am_i(&who_am_i);
	if (err) {
		printk("FAIL: no ACK on the IMU bus (err %d)\n", err);
		printk("      Check that SCL is on P0.%02d, SDA on P0.%02d, that the IMU\n"
		       "      has VDD/VDDIO, and that both lines have pull-ups.\n",
		       PIN_IMU_SCL, PIN_IMU_SDA);
		return err;
	}

	printk("WHO_AM_I   : 0x%02x (expected 0x%02x) -- %s\n",
	       who_am_i, ICM42686_WHO_AM_I_VALUE,
	       who_am_i == ICM42686_WHO_AM_I_VALUE ? "OK" : "MISMATCH");

	err = icm42686_init();
	if (err) {
		printk("FAIL: icm42686_init() returned %d\n", err);
		return err;
	}

	printk("IMU address: 0x%02x\n", icm42686_get_addr());
	printk("\n  # |    temp |      accel X/Y/Z (mg) |     gyro X/Y/Z (mdps)\n");
	printk("----+---------+-----------------------+----------------------\n");

	for (int i = 0; i < 5; i++) {
		int32_t temp_c_x100;

		err = icm42686_read_sample(&temp_raw, accel, gyro);
		if (err) {
			printk("FAIL: sample read failed (err %d)\n", err);
			return err;
		}

		/* Datasheet §11.13: T[degC] = TEMP_DATA / 132.48 + 25. */
		temp_c_x100 = ((int32_t)temp_raw * 10000) / 13248 + 2500;

		int32_t ax = ((int32_t)accel[0] * 1000) / ICM42686_ACCEL_LSB_PER_G;
		int32_t ay = ((int32_t)accel[1] * 1000) / ICM42686_ACCEL_LSB_PER_G;
		int32_t az = ((int32_t)accel[2] * 1000) / ICM42686_ACCEL_LSB_PER_G;

		int32_t gx = ((int32_t)gyro[0] * 10000) / ICM42686_GYRO_LSB_PER_DPS_X10;
		int32_t gy = ((int32_t)gyro[1] * 10000) / ICM42686_GYRO_LSB_PER_DPS_X10;
		int32_t gz = ((int32_t)gyro[2] * 10000) / ICM42686_GYRO_LSB_PER_DPS_X10;

		printk("  %d | %3d.%02d C | %6d %6d %6d | %6d %6d %6d\n",
		       i, temp_c_x100 / 100, temp_c_x100 % 100,
		       ax, ay, az, gx, gy, gz);

		/* Total acceleration should read ~1 g while the board sits still,
		 * whatever its orientation. That is the check that actually proves
		 * the MEMS is converting -- WHO_AM_I only proves the bus works.
		 */
		mag_mg = isqrt_u64((uint64_t)(ax * ax) +
				   (uint64_t)(ay * ay) +
				   (uint64_t)(az * az));

		if (i == 0) {
			first_accel[0] = accel[0];
			first_accel[1] = accel[1];
			first_accel[2] = accel[2];
		} else if (accel[0] != first_accel[0] ||
			   accel[1] != first_accel[1] ||
			   accel[2] != first_accel[2]) {
			moved = true;
		}

		k_msleep(100);
	}

	printk("\n|accel|    : %u mg -- %s\n", mag_mg,
	       (mag_mg > 700 && mag_mg < 1300)
		       ? "consistent with 1 g, sensor is converting"
		       : "NOT ~1 g (check the part is level/still, and the rails)");

	/* Identical counts on every read means the register block is handing back a
	 * frozen value rather than fresh conversions -- noise alone should move the
	 * low bits between samples.
	 */
	printk("Live data  : %s\n",
	       moved ? "samples vary between reads (ADC is running)"
		     : "IDENTICAL across all reads -- data looks frozen");

	return 0;
}

int main(void)
{
	printk("\n=== Kagafei bring-up ===\n");

	report_reset_reason();

	/* Each check reports its own pass/fail; a dead camera must not stop us from
	 * getting to the IMU, since they are on independent buses.
	 */
	(void)hm01b0_check();
	(void)icm42686_check();

	printk("\n=== done ===\n");

	return 0;
}
