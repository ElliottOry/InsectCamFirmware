#ifndef I2C_DIAG_H
#define I2C_DIAG_H

/* Electrical check of the two I2C buses before any transfer is attempted.
 *
 * A TWIM transfer that ends in -ETIMEDOUT cannot distinguish "no pull-up
 * resistor fitted" from "a slave is holding the line low" from "the device is
 * simply absent" -- in all three cases the controller just never sees the bus
 * go high.  This probes the lines directly with the SoC's own pull resistors
 * and reports which of those it actually is.
 *
 * Safe to call with the TWIM drivers already initialised: each bus is
 * disabled, probed, and restored to the exact configuration it had.
 */
void i2c_diag_all(void);

#endif /* I2C_DIAG_H */
