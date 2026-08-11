#ifndef HM01B0I2C_H_
#define HM01B0I2C_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "HM01B0Regs.h"
#include "HM01B0_def_values.h"


static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
void init_cam(void);
void hm_i2c_write(uint16_t reg, uint8_t val);
/* Verify the sensor answers on I2C (MODEL_ID_H == 0x01), running 9-clock bus
 * recovery between attempts.  A reset/halt mid-transaction leaves the HM01B0
 * holding SDA low; without recovery every later write fails silently and the
 * capture pipeline stalls forever.  Returns 0 when the sensor responds. */
int hm_cam_probe(void);



#endif /* HM01B0I2C_H_ */
