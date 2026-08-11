#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>

/* Ensure SPIS CSN pin (P0.6, the camera's LVLD line) is pulled up before the
 * SPIS driver probes.
 * Some external devices may hold the line low at reset, which causes the
 * nrfx SPIS init to fail. By configuring a pull-up early, we avoid an
 * asserted CSN during probe.
 */
static int spis_csn_pullup_init(void)
{
    const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio)) {
        return 0;
    }
    gpio_pin_configure(gpio, 6, GPIO_INPUT | GPIO_PULL_UP);
    return 0;
}

SYS_INIT(spis_csn_pullup_init, PRE_KERNEL_1, 0);

