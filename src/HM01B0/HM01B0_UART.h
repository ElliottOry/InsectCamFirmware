#ifndef HM01B0_UART_H_
#define HM01B0_UART_H_


#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include "HM01B0_def_values.h"
#include <zephyr/kernel.h>

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
void send_frame_over_uart_binary(uint8_t *image, atomic_t *lines_done);
void setup_cam_capture_fifo(struct k_fifo *fifo);
void streaming_photos_fifo(void *p1, void *p2, void *p3);


#endif /* HM01B0_UART_H_ */
