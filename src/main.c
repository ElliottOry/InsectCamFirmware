/*
 * Kagafei module: HM01B0 capture -> JPEG -> BLE live view.
 *
 * Flow: MCLK out on P0.18, sensor configured over i2c0, pixel lines land via
 * SPIS2 (PCLKO=SCK P0.07, D0=MOSI P0.20, LVLD=CSN P0.06), VSYNC on P0.08
 * frames each capture, and ble_manager streams each 160x119 8-bit grayscale
 * frame to the "insect_cam" GATT service (receiver: save_to_file_bt.py).
 * Wire format is selected by BLE_STREAM_RAW in ble_manager.c: KJPG = q90
 * on-device JPEG, ~3 fps, lossy (current); KRAW = every pixel byte verbatim,
 * lossless, ~1.1 fps.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_power.h>

#include "HM01B0/HM01B0_def_values.h"
#include "HM01B0/HM01B0_CAPTURE.h"
#include "HM01B0/HM01B0_BLE.h"
#include "imu_stream.h"
#include "i2c_diag.h"

/* Temporary IMU-only bringup: skip the camera and BLE entirely and print
 * ICM-42686 samples over RTT.  Set to 0 to restore the normal stream. */
#define IMU_ONLY_TEST 0

/* Power-floor test: measure what the BOARD draws with the firmware doing
 * nothing at all.  Phase 1: 15 s of System ON idle -- only the kernel tick and
 * the devicetree-initialized peripherals (TWIM/SPIS/pinctrl), no camera, no
 * MCLK, no BLE, no IMU.  Phase 2: System OFF forever (~0.3 uA for a bare
 * nRF52832; a power cycle restarts).  Reading the phases on the PPK:
 *   - Phase 2 above ~10 uA  -> external hardware (LED, regulator, sensor
 *     rails, pin conflict), not the nRF and not firmware.
 *   - Phase 1 mA-level but phase 2 clean -> a DT-enabled peripheral or pin
 *     state inside the nRF.
 *   - Both clean            -> the draw is in the app path; bisect there.
 * Set to 0 to restore the normal stream. */
#define POWER_FLOOR_TEST 0

struct k_fifo my_fifo_ble;

/* An all-zero RESETREAS on a repeating reset means brownout/power-on: the
 * supply is sagging, which also keeps the sensor's rails from coming up.
 * Cleared after reading, since the bits are cumulative.
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

#if POWER_FLOOR_TEST

int main(void)
{
	printk("\n=== POWER FLOOR TEST: 15 s System ON idle, then System OFF ===\n");
	report_reset_reason();

	/* Force the lowest System ON state so phase 1 is the board's best case:
	 * GPIOTE fully disabled (the SPIS driver's anomaly-109 workaround claims a
	 * channel at init, and any Task/Event-mode channel holds the 16 MHz clock
	 * on through sleep), serial peripherals off, and the camera's MCLK input
	 * driven low so it cannot float (a floating CMOS input in the sensor can
	 * draw milliamps that the normal firmware, which drives this pin, never
	 * sees). */
	NRF_GPIOTE->INTENCLR = 0xFFFFFFFF;
	for (int ch = 0; ch < 8; ch++) {
		NRF_GPIOTE->CONFIG[ch] = 0;
	}
	NRF_SPIS2->ENABLE = 0;
	NRF_TWIM0->ENABLE = 0;
	NRF_TWIM1->ENABLE = 0;
	NRF_P0->PIN_CNF[PIN_MCLK] =
		(GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
		(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
	NRF_P0->OUTCLR = BIT(PIN_MCLK);

	k_sleep(K_SECONDS(15));

	printk("Entering System OFF (power cycle to restart)\n");
	nrf_power_system_off(NRF_POWER);

	return 0;
}

#elif IMU_ONLY_TEST
#include "icm42686.h"
#include <zephyr/drivers/i2c.h>

/* A blind 1-byte read ACK-tests each address (nrfx TWIM rejects 0-length
 * transfers, so the usual empty-write probe is not an option here). */
static int i2c_scan(const struct device *dev, const char *name)
{
	int found = 0;

	printk("I2C scan %s:", name);
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b;

		if (i2c_read(dev, &b, 1, addr) == 0) {
			printk(" 0x%02X", addr);
			found++;
		}
	}
	printk("%s\n", found ? "" : " -- no ACKs");
	return found;
}

int main(void)
{
	printk("\n=== Kagafei IMU-only test (camera + BLE disabled) ===\n");
	report_reset_reason();

	i2c_diag_all();

	const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c1_dev)) {
		printk("i2c1 not ready\n");
		return -ENODEV;
	}
	/* Keep retrying so wiring/power can be wiggled while watching RTT.
	 * Both buses are scanned every pass (not one-shot at boot, which the
	 * 1 KB RTT buffer can drop before the host attaches): camera should
	 * ACK at 0x24 if its rail is up (it answers I2C without MCLK). */
	while (icm42686_init() != 0) {
		if (device_is_ready(i2c0_dev)) {
			i2c_scan(i2c0_dev, "i2c0 (camera bus)");
		}
		i2c_scan(i2c1_dev, "i2c1 (IMU bus, SCL P0.28 SDA P0.02)");
		printk("ICM-42686 not found; retrying in 2 s\n");
		k_msleep(2000);
	}

	printk("ICM-42686 found at 0x%02X, printing samples at 10 Hz\n",
	       icm42686_get_addr());

	for (uint32_t n = 0;; n++) {
		int16_t t, a[3], g[3];

		/* Re-confirm the camera bus every ~5 s while streaming. */
		if (n % 50 == 0 && device_is_ready(i2c0_dev)) {
			i2c_scan(i2c0_dev, "i2c0 (camera bus)");
		}

		if (icm42686_read_sample(&t, a, g) != 0) {
			printk("[%5u] read error\n", n);
		} else {
			/* Temp degC = raw/132.48 + 25, printed in centi-degC */
			int32_t temp_cc = (int32_t)t * 10000 / 13248 + 2500;

			printk("[%5u] accel mg: %6d %6d %6d | gyro dps*10: %6d %6d %6d | temp %d.%02u C\n",
			       n,
			       (int32_t)a[0] * 1000 / ICM42686_ACCEL_LSB_PER_G,
			       (int32_t)a[1] * 1000 / ICM42686_ACCEL_LSB_PER_G,
			       (int32_t)a[2] * 1000 / ICM42686_ACCEL_LSB_PER_G,
			       (int32_t)g[0] * 100 / ICM42686_GYRO_LSB_PER_DPS_X10,
			       (int32_t)g[1] * 100 / ICM42686_GYRO_LSB_PER_DPS_X10,
			       (int32_t)g[2] * 100 / ICM42686_GYRO_LSB_PER_DPS_X10,
			       temp_cc / 100,
			       (unsigned)((temp_cc < 0 ? -temp_cc : temp_cc) % 100));
		}
		k_msleep(100);
	}

	return 0;
}

#else /* !IMU_ONLY_TEST: normal camera + IMU + BLE stream */

int main(void)
{
	printk("\n=== Kagafei camera + BLE ===\n");
	report_reset_reason();

	/* Before any transfer: if the lines cannot idle high, every probe below
	 * would just report -ETIMEDOUT without saying why. */
	i2c_diag_all();

	if (!device_is_ready(dev_i2c.bus)   ||
	    !device_is_ready(gpio0_dev)     ||
	    !device_is_ready(spis_dev.bus)) {
		printk("Device not ready\n");
		return -ENODEV;
	}

	k_fifo_init(&my_fifo_ble);
	/* BLE-only live view: feed frames to BLE alone so the capture loop isn't
	 * gated by (or competing for CPU with) any other consumer.
	 */
	setup_cam_capture(&my_fifo_ble, NULL);
	hm_ble_init(&my_fifo_ble);

	/* Spot-check that init_cam()'s configuration actually landed. */
	uint8_t addr_readback[2];
	uint8_t val;

	addr_readback[0] = 0x03; addr_readback[1] = 0x50;
	if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0) {
		printk("QVGA win (0x0350) = 0x%02X\n", val);
	}

	addr_readback[0] = 0x03; addr_readback[1] = 0x81;
	if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0) {
		printk("BIN_RDOUT_Y (0x0381) = 0x%02X\n", val);
	}

	addr_readback[0] = 0x10; addr_readback[1] = 0x06;
	if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0) {
		printk("IMG_MODE_SEL (0x1006) = 0x%02X\n", val);
	}

	/* IMU is optional: a missing/dead ICM-42686 must not take down the camera. */
	if (imu_stream_start() != 0) {
		printk("ICM-42686 not available -- streaming camera only\n");
	}

	printk("Starting capture loop\n");
	stream_photos();

	return 0;
}

#endif /* IMU_ONLY_TEST */
