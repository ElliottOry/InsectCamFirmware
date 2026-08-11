#ifndef ICM42686_H
#define ICM42686_H

#include <stdint.h>

/* WHO_AM_I reset value, unique to the ICM-42686-P (datasheet §14.75).
 * Sibling parts differ -- the 42688-P reads 0x47 -- so this doubles as a
 * check that the part on the bus is the one we think it is.
 */
#define ICM42686_WHO_AM_I_VALUE 0x44

/* 7-bit address is 0b110100X; the LSB follows the AP_AD0 pin strap
 * (datasheet §11.2). icm42686_init() probes both, so the board does not
 * have to tell us how AP_AD0 is tied.
 */
#define ICM42686_ADDR_AD0_LOW  0x68
#define ICM42686_ADDR_AD0_HIGH 0x69

/* Full-scale settings applied by icm42686_init(), exported so callers can
 * convert raw counts without duplicating the constants.
 */
#define ICM42686_ACCEL_LSB_PER_G    2048 /* +/-16g  */
#define ICM42686_GYRO_LSB_PER_DPS_X10 164 /* +/-2000 dps -> 16.4 LSB/dps */

/* Bring the IMU up: locate it on the bus, soft reset, configure filters,
 * full-scale ranges and ODRs, and turn the accel + gyro on in low-noise mode.
 * Returns 0 on success, or a negative errno. -ENODEV means nothing on the
 * bus answered with a valid WHO_AM_I.
 */
int icm42686_init(void);

/* Power the accel + gyro (and RC oscillator) down to their ~µA standby, keeping
 * the register file, and bring them back to the init() power state.  wake()
 * includes the ~100 ms gyro spin-up wait.  Both need a successful init() first
 * and return a negative errno otherwise.
 */
int icm42686_sleep(void);
int icm42686_wake(void);

/* Reads WHO_AM_I from whichever address init() latched onto. Safe to call
 * before init(), in which case it probes the AP_AD0-low address.
 */
int icm42686_read_who_am_i(uint8_t *who_am_i);

/* The address init() found the device at, or 0 if init() has not succeeded. */
uint8_t icm42686_get_addr(void);

/* Burst-reads the 12 accel+gyro data bytes (ACCEL_DATA_X1..GYRO_DATA_Z0),
 * big-endian, X/Y/Z accel then X/Y/Z gyro. `data` must hold 12 bytes.
 */
int icm42686_get_accel_gyro(uint8_t *data);

/* Burst-reads temperature + accel + gyro (TEMP_DATA1..GYRO_DATA_Z0) and
 * unpacks them into signed counts. Any pointer may be NULL.
 * `accel` and `gyro` each take 3 int16_t (X, Y, Z).
 */
int icm42686_read_sample(int16_t *temp_raw, int16_t *accel, int16_t *gyro);

#endif /* ICM42686_H */
