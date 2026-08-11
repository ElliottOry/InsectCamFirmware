/*
* HM01B0 capture example – interrupt driven, DMA-ready version
*
* Copyright (c) 2024 Open Pixel Systems
* SPDX-License-Identifier: Apache-2.0
*/




#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>


#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>



//#include <drivers/spi/spi_nrfx_spis.h>

#include <zephyr/drivers/pwm.h>




#include "HM01B0/HM01B0_def_values.h"
#include "HM01B0/HM01B0_CAPTURE.h"
#include "HM01B0/HM01B0_BLE.h"

struct k_fifo my_fifo_uart;
struct k_fifo my_fifo_ble;

/* Frame consumer for this build:
 *   1 = UART @ 921600 on uart0 (receiver: "save_to_file copy3.py") — wire does
 *       ~92 KB/s, so a 19040 B frame takes ~207 ms => ~4.5 fps pipeline test,
 *       free of the macOS BLE radio cap.  Requires CONFIG_UART_CONSOLE=n in
 *       prj.conf (set) so printk/log bytes can't corrupt the image stream —
 *       console + logs go to RTT instead.
 *   0 = BLE live view (receiver: save_to_file_bt.py).
 * Only one consumer at a time: with both attached, the slower one gates the
 * frame rate for the other. */
#define STREAM_OVER_UART 0

int main(void)
{

    //printk("HM01B0 Capture Example\n");
#if STREAM_OVER_UART
    k_fifo_init(&my_fifo_uart);
    setup_cam_capture(&my_fifo_uart, NULL);
    setup_cam_capture_fifo(&my_fifo_uart);
#else
    k_fifo_init(&my_fifo_ble);
    /* BLE-only live view: feed frames to BLE alone so the capture loop isn't gated by
     * (or competing for CPU with) the UART image stream. */
    setup_cam_capture(&my_fifo_ble, NULL);
    hm_ble_init(&my_fifo_ble);
#endif
    /* leave this on forever */
    //spispec.config = spi_cfg_pwr1;
    //hm_ble_init(&my_fifo);
    if (!device_is_ready(uart_dev)      ||
        !device_is_ready(dev_i2c.bus)   ||
        !device_is_ready(gpio0_dev)     ||
        !device_is_ready(spis_dev.bus)) {
        printk("Device not ready\n");
        return -ENODEV;
    }


    //run_peripheral_step(0);
/* --- CSN pin -------------------------------------------------------- */
    uint8_t addr_readback[2];
    uint8_t val;

    /* BIT_CONTROL should read back 0x20 for 1-bit serial mode. */
    addr_readback[0] = 0x03; addr_readback[1] = 0x50;
    if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
        printk("QVGA win (0x0350) = 0x%02X\n", val);

    /* OSC_CLK_DIV should read back 0x00 (MSB-first, ÷8, non-gated). */
    addr_readback[0] = 0x03; addr_readback[1] = 0x81;
    if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
        printk("BIN_RDOUT_Y (0x0381) = 0x%02X\n", val);

    /* TEST_PATTERN_MODE – should be 0x11 (walking 1 enabled) */
    addr_readback[0] = 0x10; addr_readback[1] = 0x06;
    if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
        printk("IMG_MODE_SEL (0x1006) = 0x%02X\n", val);
    /* ------------------------- main loop ---------------------------------- */
    printk("Starting capture loop\n");
    stream_photos();
}
