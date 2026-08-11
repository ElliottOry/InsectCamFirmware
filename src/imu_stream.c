/* 100 Hz ICM-42686 sampler feeding a drop-oldest ring that the BLE streaming
 * thread drains into KIMU packets between camera frames (ble_stream_imu in
 * ble_manager.c).  The sensor's ODR stays at 1 kHz; reading every 10 ms is
 * the decimation down to the ~100 Hz the link carries.
 */

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>

#include "imu_stream.h"
#include "icm42686.h"

/* Connection gate (ble_manager.c): sampling pauses while no central is
 * connected -- nothing drains the ring then, so running the IMU and the 100 Hz
 * I2C traffic would only burn battery. */
extern bool ble_is_connected(void);
extern void ble_wait_for_connection(void);

#define IMU_SAMPLE_PERIOD_MS 10

/* 128 * 16 B = 2 KB: 1.28 s of backlog.  Frame gaps are 100-330 ms and the
 * capture stall watchdog is 500 ms, so only a BLE link that has been wedged
 * for over a second overflows this -- at which point the oldest samples are
 * stale and dropping them is the right call. */
#define IMU_RING_SIZE 128

static struct imu_sample ring[IMU_RING_SIZE];
static uint32_t ring_head;   /* next write slot */
static uint32_t ring_tail;   /* next read slot */
static uint32_t ring_count;
static struct k_spinlock ring_lock;

static bool imu_active;
static int16_t last_temp_raw;
static uint32_t dropped_total;
static uint32_t read_errors;

K_THREAD_STACK_DEFINE(imu_stack, 1024);
static struct k_thread imu_thread_data;

static void ring_push(const struct imu_sample *s)
{
	uint32_t dropped = 0;

	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	if (ring_count == IMU_RING_SIZE) {
		ring_tail = (ring_tail + 1U) % IMU_RING_SIZE;   /* drop oldest */
		ring_count--;
		dropped = ++dropped_total;
	}
	ring[ring_head] = *s;
	ring_head = (ring_head + 1U) % IMU_RING_SIZE;
	ring_count++;
	k_spin_unlock(&ring_lock, key);

	if (dropped != 0 && (dropped % 256U) == 1U) {
		printk("IMU ring overflow (%u dropped total)\n", dropped);
	}
}

static void ring_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	ring_head = 0;
	ring_tail = 0;
	ring_count = 0;
	k_spin_unlock(&ring_lock, key);
}

bool imu_stream_pop(struct imu_sample *out)
{
	bool ok = false;

	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	if (ring_count > 0) {
		*out = ring[ring_tail];
		ring_tail = (ring_tail + 1U) % IMU_RING_SIZE;
		ring_count--;
		ok = true;
	}
	k_spin_unlock(&ring_lock, key);

	return ok;
}

size_t imu_stream_pending(void)
{
	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	uint32_t n = ring_count;
	k_spin_unlock(&ring_lock, key);

	return n;
}

bool imu_stream_active(void)
{
	return imu_active;
}

int16_t imu_stream_last_temp_raw(void)
{
	return last_temp_raw;
}

static void imu_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	int64_t next = k_uptime_get();

	for (;;) {
		if (!ble_is_connected()) {
			/* Park: sensor to standby, drop the now-stale backlog (a
			 * reconnect must not open with seconds-old samples), and
			 * sleep until the next central. */
			icm42686_sleep();
			ring_reset();
			ble_wait_for_connection();
			icm42686_wake();
			next = k_uptime_get();
			continue;
		}

		next += IMU_SAMPLE_PERIOD_MS;
		/* Drift-free pacing; if higher-priority work held us off for a
		 * while, resync instead of burst-reading to catch up. */
		if (next < k_uptime_get() - 2 * IMU_SAMPLE_PERIOD_MS) {
			next = k_uptime_get();
		}
		k_sleep(K_TIMEOUT_ABS_MS(next));

		struct imu_sample s;
		int16_t t;

		if (icm42686_read_sample(&t, s.accel, s.gyro) == 0) {
			s.ts_ms = (uint32_t)k_uptime_get();
			last_temp_raw = t;
			ring_push(&s);
		} else if ((++read_errors % 100U) == 1U) {
			printk("IMU read error (%u total)\n", read_errors);
		}
	}
}

int imu_stream_start(void)
{
	int err = icm42686_init();

	if (err != 0) {
		return err;
	}

	imu_active = true;
	/* Prio 4: below the capture line thread (0) and the BLE threads (3),
	 * so sampling can never delay a line transfer or a notify.  Each
	 * sample carries its own timestamp, so jitter here is harmless. */
	k_thread_create(&imu_thread_data, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack),
			imu_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(4), 0, K_NO_WAIT);

	return 0;
}
