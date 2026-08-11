#ifndef HM01B0_BLE_H
#define HM01B0_BLE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "HM01B0_def_values.h"

void hm_ble_init(struct k_fifo *fifo);
void streaming_photos_ble_fifo(void *p1, void *p2, void *p3);
extern void run_peripheral_step( uint16_t seconds);
/* Stream the capture RAW (full 160x119 8-bit grayscale, uncompressed) out the TX
 * characteristic. Holds the buffer until the frame is sent. See ble_manager.c. */
extern void ble_stream_frame(const uint8_t *image, atomic_t *lines_done);
/* Drain buffered IMU samples out the TX characteristic as KIMU packets.  Only
 * the streaming thread may call this, and only between frames. See ble_manager.c. */
extern void ble_stream_imu(void);
void init_ble(void);
/* Connection gate (ble_manager.c): true while a central is connected; the wait
 * blocks until the next connect so idle threads never have to poll. */
extern bool ble_is_connected(void);
extern void ble_wait_for_connection(void);


#endif /* HM01B0_BLE_H */
