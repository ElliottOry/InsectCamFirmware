/*
 * HM01B0_CAPTURE.h
 *
 *  Created on: August 8, 2025
 *      Author: Elliott Ory
 */

#ifndef HM01B0_CAPTURE_H_
#define HM01B0_CAPTURE_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "HM01B0_def_values.h"
#include "HM01B0_I2C.h"
#include "HM01B0_SPI.h"
#include "HM01B0_CLK.h"

void setup_cam_capture(struct k_fifo *fifo1, struct k_fifo *fifo2);
static void vsync_isr(const struct device *dev,struct gpio_callback *cb,uint32_t pins);
static void line_thread(void *p1, void *p2, void *p3);
static void start_capture_fn(struct k_work *work);
void stream_photos(void);
void take_single_photo(void);
void turn_off_cam(void);
void cam_standby(void);

static const struct device *gpio0_dev  = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#endif /* HM01B0_CAPTURE_H_ */
