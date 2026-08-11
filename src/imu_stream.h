#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One decimated IMU sample: uptime timestamp plus raw counts straight from
 * the ICM-42686 (accel 2048 LSB/g, gyro 16.4 LSB/dps -- see icm42686.h). */
struct imu_sample {
	uint32_t ts_ms;
	int16_t accel[3];
	int16_t gyro[3];
};

/* Bring up the ICM-42686 on i2c1 and start the 100 Hz sampling thread.
 * Returns 0 on success, negative errno if the IMU is absent/unresponsive.
 * Failure is non-fatal: without it the app just streams camera-only. */
int imu_stream_start(void);

bool imu_stream_active(void);

size_t imu_stream_pending(void);

/* Pop the oldest buffered sample. Returns false if the ring is empty. */
bool imu_stream_pop(struct imu_sample *out);

/* Raw temperature register of the most recent sample. */
int16_t imu_stream_last_temp_raw(void);

#endif /* IMU_STREAM_H */
