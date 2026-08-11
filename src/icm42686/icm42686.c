#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>

#include "icm42686.h"
#include "icm42686_registers.h"

#include <zephyr/logging/log.h>

/****************************************/
/******* PRIVATE DATA DEFINITIONS *******/
/****************************************/
LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

/* The IMU hangs off i2c1 (TWIM1) on P0.28/SCL + P0.02/SDA. This is a different
 * peripheral instance and different pins from i2c0, which the HM01B0 owns --
 * the two buses share nothing.
 */
static const struct device *const imu_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Latched by icm42686_init() once a device answers. */
static uint8_t _addr;

/* Which register bank the device currently has selected. -1 means "unknown",
 * which forces the next set_bank() to actually write REG_BANK_SEL rather than
 * trust the cache. This matters on a warm MCU reset: the MCU restarts with the
 * cache cleared but the IMU keeps whatever bank it was left in, so assuming
 * bank 0 here would silently read the wrong registers.
 */
static int _bank = -1;

typedef enum {
    ACC_RANGE_32G = 0,
    ACC_RANGE_16G,
    ACC_RANGE_8G,
    ACC_RANGE_4G,
    ACC_RANGE_2G,
} acc_scale_type_t;

typedef enum {
    GYRO_RANGE_4000_DPS = 0,
    GYRO_RANGE_2000_DPS,
    GYRO_RANGE_1000_DPS,
    GYRO_RANGE_500_DPS,
    GYRO_RANGE_250_DPS,
    GYRO_RANGE_125_DPS,
    GYRO_RANGE_62_5_DPS,
    GYRO_RANGE_31_25_DPS,
}  gyro_scale_type_t;

typedef enum {
    ODR_32_KHZ = 0x01, // LN mode only
    ODR_16_KHZ = 0x02, // LN mode only
    ODR_8_KHZ = 0x03, // LN mode only
    ODR_4_KHZ = 0x04, // LN mode only
    ODR_2_KHZ = 0x05, // LN mode only
    ODR_1_KHZ = 0x06, // LN mode only
    ODR_200_HZ = 0x07,
    ODR_100_HZ = 0x08,
    ODR_50_HZ = 0x09,
    ODR_25_HZ = 0x0A,
    ODR_12_5_HZ = 0x0B,
    ODR_6a25_HZ = 0x0C, // LP mode only (accel only)
    ODR_3a125_HZ = 0x0D, // LP mode only (accel only)
    ODR_1a5625_HZ = 0x0E, // LP mode only (accel only)
    ODR_500_HZ = 0x0F,
} odr_type_t;

typedef enum {
    AAF_BW_42_HZ = 0,
    AAF_BW_84_HZ,
    AAF_BW_126_HZ,
    AAF_BW_170_HZ,
    AAF_BW_213_HZ,
    AAF_BW_COUNT
} aaf_bw_e;

const uint16_t aaf_table[AAF_BW_COUNT][3] = {
    {1, 1, 15}, // 42 Hz
    {2, 4, 13}, // 84 Hz
    {3, 9, 12}, // 126 Hz
    {4, 16, 11}, // 170 Hz
    {5, 25, 10}, // 213 Hz
};

// Used to convert raw accel readings to m/s^2
// static float accel_scale_factor = 0;
// Used to convert raw gyro readings to rad/s
// static float gyro_scale_factor = 0;

/****************************************/
/***** PRIVATE FUNCTION DECLARATIONS ****/
/****************************************/

static int write_register(uint8_t reg_addr, uint8_t data);
static int read_register(uint8_t reg_addr, uint8_t* data);
static int read_multiple_registers(uint8_t reg_addr, uint8_t* data, uint8_t len);
static int set_bank(int bank);
static int reset(void);
static int disable_spi_interface(void);
static int set_accel_fs(acc_scale_type_t acc_scale);
static int set_gyro_fs(gyro_scale_type_t gyro_scale);
static int set_accel_odr(odr_type_t acc_odr);
static int set_gyro_odr(odr_type_t gyro_odr);
static int set_accel_aaf(aaf_bw_e accel_aaf);
static int set_gyro_aaf(aaf_bw_e gyro_aaf);
static int set_accel_ui_filter(uint8_t order, uint8_t bw);
static int set_gyro_ui_filter(uint8_t order, uint8_t bw);
// static void calculate_acc_scale_factor(acc_scale_type_t acc_scale);
// static void calculate_gyro_scale_factor(gyro_scale_type_t gyro_scale);

/****************************************/
/***** PUBLIC FUNCTION DEFINITIONS ******/
/****************************************/

/* Try both AP_AD0 strappings and latch the one that answers with the right
 * WHO_AM_I, so the caller does not need to know how the pin is tied.
 */
static int probe_address(void)
{
    static const uint8_t candidates[] = {
        ICM42686_ADDR_AD0_LOW,
        ICM42686_ADDR_AD0_HIGH,
    };

    for (size_t i = 0; i < ARRAY_SIZE(candidates); i++)
    {
        uint8_t who_am_i = 0;

        _addr = candidates[i];
        _bank = -1;

        if (set_bank(0) != 0)
        {
            continue; /* No ACK at this address. */
        }

        if (read_register(UB0_REG_WHO_AM_I, &who_am_i) != 0)
        {
            continue;
        }

        if (who_am_i == ICM42686_WHO_AM_I_VALUE)
        {
            LOG_INF("ICM-42686-P found at 0x%02x (WHO_AM_I 0x%02x)",
                    _addr, who_am_i);
            return 0;
        }

        LOG_WRN("Device at 0x%02x answered WHO_AM_I 0x%02x, expected 0x%02x",
                _addr, who_am_i, ICM42686_WHO_AM_I_VALUE);
    }

    _addr = 0;
    _bank = -1;

    return -ENODEV;
}

int icm42686_init(void)
{
    int ret;

    if (!device_is_ready(imu_i2c))
    {
        LOG_ERR("%s is not ready", imu_i2c->name);
        return -ENODEV;
    }

    ret = probe_address();
    if (ret)
    {
        LOG_ERR("No ICM-42686-P on the bus (tried 0x%02x and 0x%02x)",
                ICM42686_ADDR_AD0_LOW, ICM42686_ADDR_AD0_HIGH);
        return ret;
    }

    ret = reset();
    if (ret)
    {
        LOG_ERR("Soft reset failed (%d)", ret);
        return ret;
    }

    /* We are on I2C, so shut SPI off before configuring anything else. */
    ret = disable_spi_interface();
    if (ret)
    {
        LOG_ERR("Failed to disable the SPI interface (%d)", ret);
        return ret;
    }

    // Set accel and gyro off, only rc osc on
    set_bank(0);
    ret = write_register(UB0_REG_PWR_MGMT0, 0x10);
    if (ret)
    {
        LOG_ERR("PWR_MGMT0 idle write failed (%d)", ret);
        return ret;
    }

    // Wait for some time
    k_msleep(100);

    // Set Anti-aliasing filters
    set_accel_aaf(AAF_BW_213_HZ);
    set_gyro_aaf(AAF_BW_213_HZ);

    // Set UI filters to order 0, bw 0
    set_accel_ui_filter(0, 0);
    set_gyro_ui_filter(0, 0);

    // Set Accel scale
    set_accel_fs(ACC_RANGE_16G);
    set_gyro_fs(GYRO_RANGE_2000_DPS);

    // calculate_acc_scale_factor(ACC_RANGE_16G);
    // calculate_gyro_scale_factor(GYRO_RANGE_2000_DPS);

    // Set ODRs
    set_accel_odr(ODR_1_KHZ);
    set_gyro_odr(ODR_1_KHZ);

    // Wait for some time
    k_msleep(100);

    // Turn on accel and gyro
    // PWR_MGMT0: IDLE=1, GYRO_MODE=11, ACCEL_MODE=11 -> both in low-noise mode
    set_bank(0);
    ret = write_register(UB0_REG_PWR_MGMT0, 0x1F);
    if (ret)
    {
        LOG_ERR("PWR_MGMT0 enable write failed (%d)", ret);
        return ret;
    }

    /* The gyro needs tens of ms to spin up before its output is meaningful. */
    k_msleep(100);

    LOG_INF("ICM-42686-P initialised: accel +/-16g, gyro +/-2000dps, 1 kHz ODR");

    return 0;
}

int icm42686_sleep(void)
{
    int ret;

    if (_addr == 0)
    {
        return -ENODEV;
    }

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    /* PWR_MGMT0 all-off: accel + gyro + RC oscillator down (~µA standby vs
     * ~mA in low-noise mode).  The register file is retained. */
    return write_register(UB0_REG_PWR_MGMT0, 0x00);
}

int icm42686_wake(void)
{
    int ret;

    if (_addr == 0)
    {
        return -ENODEV;
    }

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    /* Same power state as init(): IDLE=1, gyro + accel in low-noise mode. */
    ret = write_register(UB0_REG_PWR_MGMT0, 0x1F);
    if (ret)
    {
        return ret;
    }

    /* The gyro needs tens of ms to spin up before its output is meaningful. */
    k_msleep(100);

    return 0;
}

int icm42686_read_who_am_i(uint8_t *who_am_i)
{
    int ret;

    if (_addr == 0)
    {
        /* Not initialised yet -- fall back to the default strapping so this can
         * be used as a standalone bus check.
         */
        _addr = ICM42686_ADDR_AD0_LOW;
        _bank = -1;
    }

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    return read_register(UB0_REG_WHO_AM_I, who_am_i);
}

uint8_t icm42686_get_addr(void)
{
    return _addr;
}

int icm42686_read_sample(int16_t *temp_raw, int16_t *accel, int16_t *gyro)
{
    /* TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A) are contiguous, so one
     * burst read gets temperature, accel and gyro from the same sample.
     */
    uint8_t buf[14];
    int ret;

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    ret = read_multiple_registers(UB0_REG_TEMP_DATA1, buf, sizeof(buf));
    if (ret)
    {
        return ret;
    }

    /* SENSOR_DATA_ENDIAN defaults to big-endian (INTF_CONFIG0 bit 4). */
    if (temp_raw)
    {
        *temp_raw = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
    }

    for (int i = 0; i < 3; i++)
    {
        if (accel)
        {
            accel[i] = (int16_t) (((uint16_t) buf[2 + 2 * i] << 8) | buf[3 + 2 * i]);
        }
        if (gyro)
        {
            gyro[i] = (int16_t) (((uint16_t) buf[8 + 2 * i] << 8) | buf[9 + 2 * i]);
        }
    }

    return 0;
}

int icm42686_get_accel_gyro(uint8_t *data)
{
    set_bank(0);
    return read_multiple_registers(UB0_REG_ACCEL_DATA_X1, data, 12);
}

/*
int icm42686_get_physical_accel_gyro(float* accel, float* gyro)
{
    int ret;
    uint8_t buf[12];
    ret = icm42686_get_accel_gyro(buf);

    uint16_t raw_data[6];
    for (int i = 0; i < 6; ++i)
    {
        raw_data[i] = (((uint16_t) buf[2*i]) << 8) + ((uint16_t) buf[2*i + 1]);
    }

    for (int i = 0; i < 3; ++i)
    {
        accel[i] = ((int16_t )raw_data[i]) * accel_scale_factor;
        gyro[i] = ((int16_t )raw_data[i+3]) * gyro_scale_factor;
    }

    return ret;
}
*/
/****************************************/
/***** PRIVATE FUNCTION DEFINITIONS *****/
/****************************************/

/* Transport. Unlike SPI, I2C carries the read/write direction in the address
 * byte's R/W bit, so the register address is sent verbatim -- no 0x80 read flag
 * and no 0x7F write mask.
 */
static int write_register(uint8_t reg_addr, uint8_t data)
{
    return i2c_reg_write_byte(imu_i2c, _addr, reg_addr, data);
}

static int read_register(uint8_t reg_addr, uint8_t* data)
{
    return i2c_reg_read_byte(imu_i2c, _addr, reg_addr, data);
}

static int read_multiple_registers(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    return i2c_burst_read(imu_i2c, _addr, reg_addr, data, len);
}

static int set_bank(int bank)
{
    int ret = 0;
    if (bank != _bank)
    {
        ret = write_register(REG_BANK_SEL, (uint8_t) bank);
        if (ret)
        {
            /* Leave the cache invalid so the next call retries rather than
             * believing a bank switch that never landed.
             */
            _bank = -1;
            return ret;
        }
        _bank = bank;
    }

    return ret;
}

static int reset(void)
{
    int ret;

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    ret = write_register(UB0_REG_DEVICE_CONFIG, 0x01);
    if (ret)
    {
        return ret;
    }

    /* Datasheet §14.1: wait 1 ms after SOFT_RESET_CONFIG before touching any
     * other register. 10 ms is a comfortable margin.
     */
    k_msleep(10);

    /* Reset returns the device to bank 0, but do not trust the cache across a
     * reset -- force the next set_bank() to write.
     */
    _bank = -1;

    return 0;
}

/* Bits 1:0 of INTF_CONFIG0 (UI_SIFS_CFG) shut off the serial interface we are
 * not using: 0b10 disables SPI. With SPI left live, noise on the shared pins can
 * be latched as a SPI transaction and wedge the interface. Read-modify-write so
 * the endianness and FIFO-count bits in the upper nibble keep their defaults.
 */
static int disable_spi_interface(void)
{
    int ret;
    uint8_t reg_value;

    ret = set_bank(0);
    if (ret)
    {
        return ret;
    }

    ret = read_register(UB0_REG_INTF_CONFIG0, &reg_value);
    if (ret)
    {
        return ret;
    }

    reg_value = (reg_value & 0xFC) | 0x02;

    return write_register(UB0_REG_INTF_CONFIG0, reg_value);
}

static int set_accel_fs(acc_scale_type_t acc_scale)
{
    set_bank(0);

    uint8_t reg_value;
    read_register(UB0_REG_ACCEL_CONFIG0, &reg_value);

    reg_value = (acc_scale << 5) | (reg_value & 0x1F);

    write_register(UB0_REG_ACCEL_CONFIG0, reg_value);

    return 0;
}

static int set_gyro_fs(gyro_scale_type_t gyro_scale)
{
    set_bank(0);

    uint8_t reg_value;
    read_register(UB0_REG_GYRO_CONFIG0, &reg_value);

    reg_value = (gyro_scale << 5) | (reg_value & 0x1F);

    write_register(UB0_REG_GYRO_CONFIG0, reg_value);

    return 0;
}

static int set_accel_odr(odr_type_t acc_odr)
{
    set_bank(0);
    uint8_t reg_value;
    read_register(UB0_REG_ACCEL_CONFIG0, &reg_value);

    reg_value = acc_odr | (reg_value & 0xF0);

    write_register(UB0_REG_ACCEL_CONFIG0, reg_value);

    return 0;
}

static int set_gyro_odr(odr_type_t gyro_odr)
{
    set_bank(0);
    uint8_t reg_value;
    read_register(UB0_REG_GYRO_CONFIG0, &reg_value);

    reg_value = gyro_odr | (reg_value & 0xF0);

    write_register(UB0_REG_GYRO_CONFIG0, reg_value);

    return 0;
}

static int set_accel_aaf(aaf_bw_e accel_aaf)
{
    set_bank(2);
    uint8_t value = 0;
    read_register(UB2_REG_ACCEL_CONFIG_STATIC2, &value);
    // Enable AAF without touching bit 7
    value &= 0x80;
    // Set ACCEL_AAF_DELT
    value |= ((uint8_t) aaf_table[accel_aaf][0]) << 1;
    write_register(UB2_REG_ACCEL_CONFIG_STATIC2, value);

    // Set bits 7:0 of ACCEL_AAF_DELTSQR
    read_register(UB2_REG_ACCEL_CONFIG_STATIC3, &value);
    value = (uint8_t) (aaf_table[accel_aaf][1] & 0x00FF);
    write_register(UB2_REG_ACCEL_CONFIG_STATIC3, value);

    // Set bits 11:8 of ACCEL_AAF_DELTSQR
    read_register(UB2_REG_ACCEL_CONFIG_STATIC4, &value);
    value = (uint8_t) (aaf_table[accel_aaf][1] >> 8);
    // Set ACCEL_AAF_BITSHIFT
    value += ((uint8_t) aaf_table[accel_aaf][2]) << 4;
    write_register(UB2_REG_ACCEL_CONFIG_STATIC4, value);

    return 0;
}

static int set_gyro_aaf(aaf_bw_e gyro_aaf)
{
    set_bank(1);

    // Enable AAF and disable NF
    uint8_t reg_value;
    read_register(UB1_REG_GYRO_CONFIG_STATIC2, &reg_value);
    reg_value = 0xFD & reg_value;
    reg_value = 0x01 | reg_value;
    write_register(UB1_REG_GYRO_CONFIG_STATIC2, reg_value);

    // Set GYRO_AAF_DELT
    read_register(UB1_REG_GYRO_CONFIG_STATIC3, &reg_value);
    reg_value = ((uint8_t) aaf_table[gyro_aaf][0]) | (reg_value & 0xC0);
    write_register(UB1_REG_GYRO_CONFIG_STATIC3, reg_value);

    // Set bits 7:0 of GYRO_AAF_DELTSQR
    reg_value = (uint8_t) (aaf_table[gyro_aaf][1] & 0x00FF);
    write_register(UB1_REG_GYRO_CONFIG_STATIC4, reg_value);

    // Set bits 11:8 of GYRO_AAF_DELTSQR
    reg_value = (uint8_t) (aaf_table[gyro_aaf][1] >> 8);
    // Set GYRO_AAF_BITSHIFT
    reg_value += ((uint8_t) aaf_table[gyro_aaf][2]) << 4;
    write_register(UB1_REG_GYRO_CONFIG_STATIC5, reg_value);

    return 0;
}

static int set_accel_ui_filter(uint8_t order, uint8_t bw)
{
    set_bank(0);
    // Filter order
    uint8_t reg_value;
    read_register(UB0_REG_ACCEL_CONFIG1, &reg_value);
    reg_value = order | (reg_value & 0xE7);
    write_register(UB0_REG_ACCEL_CONFIG1, reg_value);

    // Filter bandwidth
    read_register(UB0_REG_GYRO_ACCEL_CONFIG0, &reg_value);
    reg_value = (bw << 4) | (reg_value & 0x0F);
    write_register(UB0_REG_GYRO_ACCEL_CONFIG0, reg_value);

    return 0;
}

static int set_gyro_ui_filter(uint8_t order, uint8_t bw)
{
    set_bank(0);
    // Filter order
    uint8_t reg_value;
    read_register(UB0_REG_GYRO_CONFIG1, &reg_value);
    reg_value = order | (reg_value & 0xF3);
    write_register(UB0_REG_GYRO_CONFIG1, reg_value);

    // Filter bandwidth
    read_register(UB0_REG_GYRO_ACCEL_CONFIG0, &reg_value);
    reg_value = bw | (reg_value & 0xF0);
    write_register(UB0_REG_GYRO_ACCEL_CONFIG0, reg_value);

    return 0;
}

/*
static void calculate_acc_scale_factor(acc_scale_type_t acc_scale)
{
    accel_scale_factor = 9.81;
    switch(acc_scale)
    {
        case ACC_RANGE_16G:
            accel_scale_factor *= 1.0/2048.0;
            break;
        case ACC_RANGE_8G:
            accel_scale_factor *= 1.0/4096.0;
            break;
        case ACC_RANGE_4G:
            accel_scale_factor *= 1.0/8192.0;
            break;
        case ACC_RANGE_2G:
            accel_scale_factor *= 1.0/16384.0;
            break;
    }
}

static void calculate_gyro_scale_factor(gyro_scale_type_t gyro_scale)
{
    gyro_scale_factor = 3.1415926535898/180.0; // rad/s
    switch(gyro_scale)
    {
        case GYRO_RANGE_2000_DPS:
            gyro_scale_factor *= 1.0/16.4;
            break;
        case GYRO_RANGE_1000_DPS:
            gyro_scale_factor *= 1.0/32.8;
            break;
        case GYRO_RANGE_500_DPS:
            gyro_scale_factor *= 1.0/65.5;
            break;
        case GYRO_RANGE_250_DPS:
            gyro_scale_factor *= 1.0/131.0;
            break;
        case GYRO_RANGE_125_DPS:
            gyro_scale_factor *= 1.0/262.0;
            break;
        case GYRO_RANGE_62_5_DPS:
            gyro_scale_factor *= 1.0/524.3;
            break;
        case GYRO_RANGE_31_25_DPS:
            gyro_scale_factor *= 1.0/1048.6;
            break;
        case GYRO_RANGE_15_625_DPS:
            gyro_scale_factor *= 1.0/2097.2;
            break;
    }
}
*/