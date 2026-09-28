#include "mpu6050.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>


/*
 * MPU6050 registers
 */
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C

#define REG_ACCEL_XOUT_H     0x3B
#define REG_TEMP_OUT_H       0x41
#define REG_GYRO_XOUT_H      0x43

#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75


#define MPU6050_WHO_AM_I_VALUE   0x68


/*
 * Scale factors
 */
static float accel_scale(mpu6050_accel_range_t range)
{
    switch (range)
    {
        case MPU6050_ACCEL_RANGE_2G:
            return 16384.0f;

        case MPU6050_ACCEL_RANGE_4G:
            return 8192.0f;

        case MPU6050_ACCEL_RANGE_8G:
            return 4096.0f;

        case MPU6050_ACCEL_RANGE_16G:
            return 2048.0f;

        default:
            return 0.0f;
    }
}


static float gyro_scale(mpu6050_gyro_range_t range)
{
    switch (range)
    {
        case MPU6050_GYRO_RANGE_250DPS:
            return 131.0f;

        case MPU6050_GYRO_RANGE_500DPS:
            return 65.5f;

        case MPU6050_GYRO_RANGE_1000DPS:
            return 32.8f;

        case MPU6050_GYRO_RANGE_2000DPS:
            return 16.4f;

        default:
            return 0.0f;
    }
}


/*
 * Validate I2C address.
 */
static int validate_address(uint8_t address)
{
    if (address != MPU6050_ADDRESS_LOW &&
        address != MPU6050_ADDRESS_HIGH)
    {
        return EINVAL;
    }

    return EOK;
}


/*
 * Write one MPU6050 register.
 */
static int write_register(mpu6050_t *dev,
                          uint8_t reg,
                          uint8_t value)
{
    struct
    {
        i2c_send_t send;
        uint8_t data[2];

    } packet;

    memset(&packet, 0, sizeof(packet));

    packet.send.slave.addr = dev->address;
    packet.send.slave.fmt = I2C_ADDRFMT_7BIT;
    packet.send.len = 2;
    packet.send.stop = 1;

    packet.data[0] = reg;
    packet.data[1] = value;

    return devctl(dev->fd,
                  DCMD_I2C_SEND,
                  &packet,
                  sizeof(packet),
                  NULL);
}


/*
 * Read multiple MPU6050 registers.
 */
static int read_registers(mpu6050_t *dev,
                           uint8_t reg,
                           uint8_t *data,
                           uint8_t length)
{
    if (data == NULL || length == 0)
        return EINVAL;

    /*
     * MPU6050 maximum data required by this driver
     * is 14 bytes:
     *
     * ACCEL  = 6
     * TEMP   = 2
     * GYRO   = 6
     */
    if (length > 14)
        return EINVAL;


    struct
    {
        i2c_sendrecv_t x;
        uint8_t data[14];

    } packet;

    memset(&packet, 0, sizeof(packet));

    packet.x.slave.addr = dev->address;
    packet.x.slave.fmt = I2C_ADDRFMT_7BIT;

    packet.x.send_len = 1;
    packet.x.recv_len = length;

    packet.x.stop = 1;

    packet.data[0] = reg;


    int status = devctl(dev->fd,
                        DCMD_I2C_SENDRECV,
                        &packet,
                        sizeof(packet),
                        NULL);

    if (status != EOK)
        return status;


    memcpy(data,
           packet.data,
           length);

    return EOK;
}


/*
 * Convert two bytes to signed 16-bit value.
 */
static int16_t make_int16(uint8_t high,
                          uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}


/*
 * Default configuration.
 */
int mpu6050_config_default(mpu6050_config_t *config)
{
    if (config == NULL)
        return EINVAL;

    config->i2c_path = MPU6050_DEFAULT_I2C_PATH;

    config->address = MPU6050_DEFAULT_ADDRESS;

    config->accel_range =
        MPU6050_ACCEL_RANGE_2G;

    config->gyro_range =
        MPU6050_GYRO_RANGE_250DPS;

    config->sample_rate_divider = 7;

    /*
     * DLPF = 0
     *
     * This is a reasonable basic default.
     */
    config->dlpf_config = 0;

    return EOK;
}


/*
 * Change configured I2C address.
 */
int mpu6050_set_address(mpu6050_config_t *config,
                        uint8_t address)
{
    if (config == NULL)
        return EINVAL;

    int status = validate_address(address);

    if (status != EOK)
        return status;

    config->address = address;

    return EOK;
}


/*
 * Initialize MPU6050.
 */
int mpu6050_init(mpu6050_t *dev,
                 const mpu6050_config_t *config)
{
    if (dev == NULL || config == NULL)
        return EINVAL;

    if (config->i2c_path == NULL)
        return EINVAL;

    if (validate_address(config->address) != EOK)
        return EINVAL;

    if (accel_scale(config->accel_range) == 0.0f)
        return EINVAL;

    if (gyro_scale(config->gyro_range) == 0.0f)
        return EINVAL;

    if (config->dlpf_config > 6)
        return EINVAL;


    memset(dev, 0, sizeof(*dev));

    dev->fd = -1;

    dev->address = config->address;

    dev->accel_range =
        config->accel_range;

    dev->gyro_range =
        config->gyro_range;

    dev->sample_rate_divider =
        config->sample_rate_divider;

    dev->dlpf_config =
        config->dlpf_config;


    int status = pthread_mutex_init(&dev->lock, NULL);

    if (status != EOK)
        return status;


    dev->fd = open(config->i2c_path, O_RDWR);

    if (dev->fd == -1)
    {
        status = errno;

        pthread_mutex_destroy(&dev->lock);

        return status;
    }


    /*
     * Wake MPU6050.
     */
    status = write_register(dev,
                            REG_PWR_MGMT_1,
                            0x00);

    if (status != EOK)
        goto fail;


    /*
     * Small startup delay.
     */
    usleep(1000);


    /*
     * Configure sample rate.
     */
    status = write_register(dev,
                            REG_SMPLRT_DIV,
                            dev->sample_rate_divider);

    if (status != EOK)
        goto fail;


    /*
     * Configure DLPF.
     */
    status = write_register(dev,
                            REG_CONFIG,
                            dev->dlpf_config);

    if (status != EOK)
        goto fail;


    /*
     * Configure accelerometer range.
     *
     * AFS_SEL occupies bits 4:3.
     */
    status = write_register(
        dev,
        REG_ACCEL_CONFIG,
        ((uint8_t)dev->accel_range << 3));

    if (status != EOK)
        goto fail;


    /*
     * Configure gyroscope range.
     *
     * FS_SEL occupies bits 4:3.
     */
    status = write_register(
        dev,
        REG_GYRO_CONFIG,
        ((uint8_t)dev->gyro_range << 3));

    if (status != EOK)
        goto fail;


    /*
     * Verify device identity.
     */
    status = mpu6050_check_device(dev);

    if (status != EOK)
        goto fail;


    dev->initialized = 1;

    return EOK;


fail:

    close(dev->fd);

    dev->fd = -1;

    pthread_mutex_destroy(&dev->lock);

    return status;
}


/*
 * Initialize with defaults.
 */
int mpu6050_init_default(mpu6050_t *dev)
{
    if (dev == NULL)
        return EINVAL;

    mpu6050_config_t config;

    int status =
        mpu6050_config_default(&config);

    if (status != EOK)
        return status;

    return mpu6050_init(dev, &config);
}


/*
 * Check WHO_AM_I.
 */
int mpu6050_check_device(mpu6050_t *dev)
{
    if (dev == NULL)
        return EINVAL;

    if (dev->fd < 0)
        return ENODEV;


    uint8_t value = 0;

    int status =
        read_registers(dev,
                       REG_WHO_AM_I,
                       &value,
                       1);

    if (status != EOK)
        return status;


    if (value != MPU6050_WHO_AM_I_VALUE)
        return ENODEV;


    return EOK;
}


/*
 * Read accelerometer.
 */
int mpu6050_read_accel(mpu6050_t *dev,
                       mpu6050_accel_t *accel)
{
    if (dev == NULL || accel == NULL)
        return EINVAL;

    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    uint8_t data[6];

    status = read_registers(dev,
                            REG_ACCEL_XOUT_H,
                            data,
                            6);

    if (status == EOK)
    {
        int16_t x =
            make_int16(data[0], data[1]);

        int16_t y =
            make_int16(data[2], data[3]);

        int16_t z =
            make_int16(data[4], data[5]);


        float scale =
            accel_scale(dev->accel_range);


        accel->x = (float)x / scale;
        accel->y = (float)y / scale;
        accel->z = (float)z / scale;
    }


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Read gyroscope.
 */
int mpu6050_read_gyro(mpu6050_t *dev,
                      mpu6050_gyro_t *gyro)
{
    if (dev == NULL || gyro == NULL)
        return EINVAL;

    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    uint8_t data[6];

    status = read_registers(dev,
                            REG_GYRO_XOUT_H,
                            data,
                            6);

    if (status == EOK)
    {
        int16_t x =
            make_int16(data[0], data[1]);

        int16_t y =
            make_int16(data[2], data[3]);

        int16_t z =
            make_int16(data[4], data[5]);


        float scale =
            gyro_scale(dev->gyro_range);


        gyro->x = (float)x / scale;
        gyro->y = (float)y / scale;
        gyro->z = (float)z / scale;
    }


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Read temperature.
 */
int mpu6050_read_temperature(mpu6050_t *dev,
                             float *temperature)
{
    if (dev == NULL || temperature == NULL)
        return EINVAL;

    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    uint8_t data[2];

    status = read_registers(dev,
                            REG_TEMP_OUT_H,
                            data,
                            2);

    if (status == EOK)
    {
        int16_t raw =
            make_int16(data[0], data[1]);


        /*
         * MPU6050 temperature conversion.
         */
        *temperature =
            ((float)raw / 340.0f) + 36.53f;
    }


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Read accelerometer + temperature + gyro
 * using ONE I2C transaction.
 */
int mpu6050_read_all(mpu6050_t *dev,
                     mpu6050_data_t *data)
{
    if (dev == NULL || data == NULL)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    uint8_t raw[14];

    status = read_registers(dev,
                            REG_ACCEL_XOUT_H,
                            raw,
                            14);

    if (status == EOK)
    {
        int16_t accel_x =
            make_int16(raw[0], raw[1]);

        int16_t accel_y =
            make_int16(raw[2], raw[3]);

        int16_t accel_z =
            make_int16(raw[4], raw[5]);


        int16_t temperature =
            make_int16(raw[6], raw[7]);


        int16_t gyro_x =
            make_int16(raw[8], raw[9]);

        int16_t gyro_y =
            make_int16(raw[10], raw[11]);

        int16_t gyro_z =
            make_int16(raw[12], raw[13]);


        float accel_factor =
            accel_scale(dev->accel_range);

        float gyro_factor =
            gyro_scale(dev->gyro_range);


        data->accel.x =
            (float)accel_x / accel_factor;

        data->accel.y =
            (float)accel_y / accel_factor;

        data->accel.z =
            (float)accel_z / accel_factor;


        data->temperature =
            ((float)temperature / 340.0f)
            + 36.53f;


        data->gyro.x =
            (float)gyro_x / gyro_factor;

        data->gyro.y =
            (float)gyro_y / gyro_factor;

        data->gyro.z =
            (float)gyro_z / gyro_factor;
    }


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Set accelerometer range.
 */
int mpu6050_set_accel_range(mpu6050_t *dev,
                            mpu6050_accel_range_t range)
{
    if (dev == NULL)
        return EINVAL;

    if (accel_scale(range) == 0.0f)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_ACCEL_CONFIG,
        ((uint8_t)range << 3));


    if (status == EOK)
        dev->accel_range = range;


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Set gyroscope range.
 */
int mpu6050_set_gyro_range(mpu6050_t *dev,
                           mpu6050_gyro_range_t range)
{
    if (dev == NULL)
        return EINVAL;

    if (gyro_scale(range) == 0.0f)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_GYRO_CONFIG,
        ((uint8_t)range << 3));


    if (status == EOK)
        dev->gyro_range = range;


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Set sample rate divider.
 */
int mpu6050_set_sample_rate_divider(mpu6050_t *dev,
                                    uint8_t divider)
{
    if (dev == NULL)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_SMPLRT_DIV,
        divider);


    if (status == EOK)
        dev->sample_rate_divider = divider;


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Set digital low-pass filter.
 */
int mpu6050_set_dlpf(mpu6050_t *dev,
                     uint8_t config)
{
    if (dev == NULL)
        return EINVAL;

    if (config > 6)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_CONFIG,
        config);


    if (status == EOK)
        dev->dlpf_config = config;


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Put MPU6050 into sleep.
 */
int mpu6050_sleep(mpu6050_t *dev)
{
    if (dev == NULL)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_PWR_MGMT_1,
        0x40);


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Wake MPU6050.
 */
int mpu6050_wakeup(mpu6050_t *dev)
{
    if (dev == NULL)
        return EINVAL;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (!dev->initialized)
    {
        pthread_mutex_unlock(&dev->lock);
        return ENODEV;
    }


    status = write_register(
        dev,
        REG_PWR_MGMT_1,
        0x00);


    pthread_mutex_unlock(&dev->lock);

    return status;
}


/*
 * Deinitialize driver.
 */
int mpu6050_deinit(mpu6050_t *dev)
{
    if (dev == NULL)
        return EINVAL;


    /*
     * If mutex was never initialized, don't touch it.
     */
    if (!dev->initialized && dev->fd < 0)
        return EOK;


    int status =
        pthread_mutex_lock(&dev->lock);

    if (status != EOK)
        return status;


    if (dev->fd >= 0)
    {
        close(dev->fd);
        dev->fd = -1;
    }


    dev->initialized = 0;


    pthread_mutex_unlock(&dev->lock);

    pthread_mutex_destroy(&dev->lock);


    return EOK;
}
