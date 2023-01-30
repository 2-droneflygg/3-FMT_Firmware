/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <firmament.h>

#include "board_device.h"
#include "hal/accel/accel.h"
#include "hal/gyro/gyro.h"
#include "hal/spi/spi.h"

#define DRV_DBG(...)

// 24 MHz max SPI frequency
#define ICM426XX_MAX_SPI_CLK_HZ 24000000

#define ICM426XX_RA_PWR_MGMT0                       0x4E
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (14 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (14 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F

#define ICM426XX_RA_INT_CONFIG                      0x14
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define icm42688_ONE_G 9.80665f

#define M_PI_F 3.1415926f

typedef enum {
    ODR_CONFIG_8K = 0,
    ODR_CONFIG_4K,
    ODR_CONFIG_2K,
    ODR_CONFIG_1K,
    ODR_CONFIG_COUNT
} odrConfig_e;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible output data rates (ODRs)
static uint8_t odrLUT[ODR_CONFIG_COUNT] = {  // see GYRO_ODR in section 5.6
    [ODR_CONFIG_8K] = 3,
    [ODR_CONFIG_4K] = 4,
    [ODR_CONFIG_2K] = 5,
    [ODR_CONFIG_1K] = 6,
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs
static aafConfig_t aafLUT[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

static rt_device_t spi_dev;
static float _gyro_range_scale;
static float _accel_range_scale;

static aafConfig_t getGyroAafConfig(void)
{
    return aafLUT[AAF_CONFIG_258HZ];
}

static rt_err_t _read_reg(rt_uint8_t reg, rt_uint8_t* buff)
{
    rt_err_t res;
    res = spi_read_reg8(spi_dev, reg, buff);
    return res;
}

static rt_err_t read_multi_reg(rt_uint8_t reg, rt_uint8_t* buff, uint8_t len)
{
    rt_err_t res;
    res = spi_read_multi_reg8(spi_dev, reg, buff,len);
    return res;
}

static rt_err_t _write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_err_t ret = RT_EOK; 
    ret = spi_write_reg8(spi_dev,reg,val);
    return ret;
}

static rt_err_t _write_checked_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_uint8_t r_buff;
    rt_err_t res = RT_EOK;

    res |= _write_reg(reg, val);
    res |= _read_reg(reg, &r_buff);

    if (r_buff != val || res != RT_EOK) {
        return RT_ERROR;
    }

    return RT_EOK;
}

/* Re-implement this function to define customized rotation */
RT_WEAK void icm42688_rotate_to_ned(float* val)
{
    /* do nothing */
}

static rt_err_t icm42688_gyr_read_raw(int16_t gyr[3])
{
    rt_err_t res;
    uint16_t raw[3];
    res = read_multi_reg(ICM426XX_RA_GYRO_DATA_X1, (uint8_t*)raw, 6);
    // big-endian to little-endian
    gyr[0] = int16_t_from_bytes((uint8_t*)&raw[0]);
    gyr[1] = int16_t_from_bytes((uint8_t*)&raw[1]);
    gyr[2] = int16_t_from_bytes((uint8_t*)&raw[2]);

    return res;
}

static rt_err_t icm42688_gyr_read_rad(float gyr[3])
{
    int16_t gyr_raw[3];
    rt_err_t res;
    res = icm42688_gyr_read_raw(gyr_raw);

    gyr[0] = _gyro_range_scale * gyr_raw[0];
    gyr[1] = _gyro_range_scale * gyr_raw[1];
    gyr[2] = _gyro_range_scale * gyr_raw[2];

    icm42688_rotate_to_ned(gyr);

    return res;
}

static rt_err_t icm42688_acc_read_raw(int16_t acc[3])
{
    int16_t raw[3];
    rt_err_t res;
    res = read_multi_reg(ICM426XX_RA_ACCEL_DATA_X1, (rt_uint8_t*)raw, 6);
    // big-endian to little-endian
    acc[0] = int16_t_from_bytes((uint8_t*)&raw[0]);
    acc[1] = int16_t_from_bytes((uint8_t*)&raw[1]);
    acc[2] = int16_t_from_bytes((uint8_t*)&raw[2]);

    return res;
}

static rt_err_t icm42688_acc_read_m_s2(float acc[3])
{
    int16_t acc_raw[3];
    rt_err_t res;
    res = icm42688_acc_read_raw(acc_raw);

    acc[0] = _accel_range_scale * acc_raw[0];
    acc[1] = _accel_range_scale * acc_raw[1];
    acc[2] = _accel_range_scale * acc_raw[2];

    icm42688_rotate_to_ned(acc);

    return res;
}

static rt_err_t icm42688regInit(void)
{
    rt_err_t ret = RT_EOK;
    ret |= _write_reg(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    systime_udelay(10000);

    ret |= _write_reg(ICM426XX_RA_GYRO_CONFIG0,(3 - INV_FSR_2000DPS) << 5 | (odrLUT[ODR_CONFIG_1K] & 0x0F));
    systime_udelay(10000);

    ret |= _write_reg(ICM426XX_RA_ACCEL_CONFIG0, (3 - INV_FSR_16G) << 5 | (odrLUT[ODR_CONFIG_1K] & 0x0F));
    systime_udelay(10000);

    // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    aafConfig_t aafConfig = getGyroAafConfig();
    ret |= _write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
    ret |= _write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    ret |= _write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
    aafConfig = aafLUT[AAF_CONFIG_258HZ];
    ret |= _write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
    ret |= _write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
    ret |= _write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure gyro and acc UI Filters
    ret |= _write_reg(ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    ret |= _write_reg(ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    ret |= _write_reg(ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);

    ret |= _write_reg(ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value;
    ret |= _read_reg(ICM426XX_RA_INT_CONFIG1,&intConfig1Value);
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

    ret |= _write_reg(ICM426XX_RA_INT_CONFIG1, intConfig1Value);
    return ret;
}

static rt_err_t low_level_init(void)
{
    uint8_t dev_id;
    rt_err_t ret = RT_EOK;
    /* init spi bus */
    ret |= rt_device_open(spi_dev, RT_DEVICE_OFLAG_RDWR);

    ret |= spi_read_reg8(spi_dev, 0x75, &dev_id);

    printf("icm42688 chip id:%x\n", dev_id);

    ret |= icm42688regInit();

    float max_dps = 2000.0;
    _gyro_range_scale = (M_PI_F / 180.0f) / (32768.0f / (float)max_dps);
    float lsb_per_g =2048;
    _accel_range_scale = (icm42688_ONE_G / lsb_per_g);

    return ret;
}

static rt_err_t accel_config(accel_dev_t accel, const struct accel_configure* cfg)
{
    rt_err_t ret = RT_EOK;

    // if (cfg == RT_NULL) {
    //     return RT_EINVAL;
    // }

    // ret |= _set_accel_range(cfg->acc_range_g);

    // ret |= _set_sample_rate(cfg->sample_rate_hz);

    // ret |= _set_dlpf_filter(cfg->dlpf_freq_hz);

    // accel->config = *cfg;



    return ret;
}

static rt_err_t gyro_config(gyro_dev_t gyro, const struct gyro_configure* cfg)
{
    rt_err_t ret = RT_EOK;

    // if (cfg == RT_NULL) {
    //     return RT_EINVAL;
    // }

    // ret |= _set_gyro_range(cfg->gyro_range_dps);

    // ret |= _set_sample_rate(cfg->sample_rate_hz);

    // ret |= _set_dlpf_filter(cfg->dlpf_freq_hz);

    // gyro->config = *cfg;


    return ret;
}

static rt_err_t gyro_control(gyro_dev_t gyro, int cmd, void* arg)
{
    return RT_EOK;
}

static rt_size_t gyro_read(gyro_dev_t gyro, rt_off_t pos, void* data, rt_size_t size)
{
    if (data == RT_NULL) {
        return 0;
    }

    if (icm42688_gyr_read_rad(((float*)data)) != RT_EOK) {
        return 0;
    }

    return size;
}

const static struct gyro_ops _gyro_ops = {
    gyro_config,
    gyro_control,
    gyro_read,
};

static rt_err_t accel_control(accel_dev_t accel, int cmd, void* arg)
{
    return RT_EOK;
}

static rt_size_t accel_read(accel_dev_t accel, rt_off_t pos, void* data, rt_size_t size)
{
    if (data == RT_NULL) {
        return 0;
    }

    if (icm42688_acc_read_m_s2(((float*)data)) != RT_EOK) {
        return 0;
    }

    return size;
}

const static struct accel_ops _accel_ops = {
    accel_config,
    accel_control,
    accel_read,
};

rt_err_t drv_icm42688_init(const char* spi_dev_name, const char* gyro_dev_name, const char* accel_dev_name)
{
    /* Initialize gyroscope */

    rt_err_t ret = RT_EOK;

    spi_dev = rt_device_find(spi_dev_name);
    RT_ASSERT(spi_dev != NULL);

    static struct accel_device accel_dev = {
        .ops = &_accel_ops,
        .config = ACCEL_CONFIG_DEFAULT,
        .bus_type = GYRO_SPI_BUS_TYPE
    };
    static struct gyro_device gyro_dev = {
        .ops = &_gyro_ops,
        .config = GYRO_CONFIG_DEFAULT,
        .bus_type = GYRO_SPI_BUS_TYPE
    };

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 4000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        ret |= rt_spi_configure(spi_device_t, &cfg);
    }
    /* device low-level init */
    ret |= low_level_init();

    /* register gyro hal device */
    ret |= hal_gyro_register(&gyro_dev, gyro_dev_name, RT_DEVICE_FLAG_RDWR, RT_NULL);

    /* register accel hal device */
    ret |= hal_accel_register(&accel_dev, accel_dev_name, RT_DEVICE_FLAG_RDWR, RT_NULL);

    return RT_EOK;
}
