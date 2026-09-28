#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <pthread.h>

/*
 * Default MPU6050 configuration
 */
#define MPU6050_DEFAULT_I2C_PATH   "/dev/i2c1"
#define MPU6050_DEFAULT_ADDRESS    0x68

/*
 * MPU6050 I2C address range
 *
 * AD0 = LOW  -> 0x68
 * AD0 = HIGH -> 0x69
 */
#define MPU6050_ADDRESS_LOW        0x68
#define MPU6050_ADDRESS_HIGH       0x69


/*
 * Accelerometer full-scale ranges
 */
typedef enum
{
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G,
    MPU6050_ACCEL_RANGE_8G,
    MPU6050_ACCEL_RANGE_16G

} mpu6050_accel_range_t;


/*
 * Gyroscope full-scale ranges
 */
typedef enum
{
    MPU6050_GYRO_RANGE_250DPS = 0,
    MPU6050_GYRO_RANGE_500DPS,
    MPU6050_GYRO_RANGE_1000DPS,
    MPU6050_GYRO_RANGE_2000DPS

} mpu6050_gyro_range_t;


/*
 * Accelerometer data in g.
 */
typedef struct
{
    float x;
    float y;
    float z;

} mpu6050_accel_t;


/*
 * Gyroscope data in degrees/second.
 */
typedef struct
{
    float x;
    float y;
    float z;

} mpu6050_gyro_t;


/*
 * Complete sensor sample.
 *
 * One read obtains:
 *
 * Accelerometer
 * Temperature
 * Gyroscope
 */
typedef struct
{
    mpu6050_accel_t accel;
    mpu6050_gyro_t gyro;

    float temperature;

} mpu6050_data_t;


/*
 * Driver configuration.
 */
typedef struct
{
    const char *i2c_path;

    uint8_t address;

    mpu6050_accel_range_t accel_range;

    mpu6050_gyro_range_t gyro_range;

    uint8_t sample_rate_divider;

    uint8_t dlpf_config;

} mpu6050_config_t;


/*
 * MPU6050 driver object.
 *
 * Do not directly modify members after initialization.
 */
typedef struct
{
    int fd;

    uint8_t address;

    mpu6050_accel_range_t accel_range;

    mpu6050_gyro_range_t gyro_range;

    uint8_t sample_rate_divider;

    uint8_t dlpf_config;

    int initialized;

    pthread_mutex_t lock;

} mpu6050_t;


/*
 * Initialize configuration with safe defaults.
 */
int mpu6050_config_default(mpu6050_config_t *config);


/*
 * Change I2C address before initialization.
 *
 * Valid addresses:
 *
 * 0x68
 * 0x69
 */
int mpu6050_set_address(mpu6050_config_t *config,
                        uint8_t address);


/*
 * Initialize MPU6050 using supplied configuration.
 */
int mpu6050_init(mpu6050_t *dev,
                 const mpu6050_config_t *config);


/*
 * Initialize using default configuration.
 *
 * Default:
 *
 * I2C path = /dev/i2c1
 * address  = 0x68
 * accel    = +/-2g
 * gyro     = +/-250 dps
 */
int mpu6050_init_default(mpu6050_t *dev);


/*
 * Check MPU6050 identity.
 *
 * Returns EOK if device responds correctly.
 */
int mpu6050_check_device(mpu6050_t *dev);


/*
 * Read accelerometer only.
 *
 * Values are returned in g.
 */
int mpu6050_read_accel(mpu6050_t *dev,
                       mpu6050_accel_t *accel);


/*
 * Read gyroscope only.
 *
 * Values are returned in degrees/second.
 */
int mpu6050_read_gyro(mpu6050_t *dev,
                      mpu6050_gyro_t *gyro);


/*
 * Read internal temperature.
 *
 * Temperature returned in Celsius.
 */
int mpu6050_read_temperature(mpu6050_t *dev,
                              float *temperature);


/*
 * Read accelerometer, gyro and temperature
 * in one I2C transaction.
 */
int mpu6050_read_all(mpu6050_t *dev,
                     mpu6050_data_t *data);


/*
 * Change accelerometer range.
 */
int mpu6050_set_accel_range(mpu6050_t *dev,
                            mpu6050_accel_range_t range);


/*
 * Change gyroscope range.
 */
int mpu6050_set_gyro_range(mpu6050_t *dev,
                           mpu6050_gyro_range_t range);


/*
 * Change sample-rate divider.
 *
 * Sample rate = Gyroscope output rate /
 *               (1 + divider)
 *
 * For DLPF enabled, gyro output rate is normally 1 kHz.
 */
int mpu6050_set_sample_rate_divider(mpu6050_t *dev,
                                    uint8_t divider);


/*
 * Configure digital low-pass filter.
 *
 * Valid values: 0 to 6.
 */
int mpu6050_set_dlpf(mpu6050_t *dev,
                     uint8_t config);


/*
 * Put MPU6050 into sleep mode.
 */
int mpu6050_sleep(mpu6050_t *dev);


/*
 * Wake MPU6050.
 */
int mpu6050_wakeup(mpu6050_t *dev);


/*
 * Close I2C device and release driver resources.
 */
int mpu6050_deinit(mpu6050_t *dev);

#endif
