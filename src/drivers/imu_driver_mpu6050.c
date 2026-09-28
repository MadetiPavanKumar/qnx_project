#include <stdio.h>
#include "imu_driver.h"
#include "mpu6050.h"

/* THIS FILE is the only place that knows about mpu6050.h. Everything
   else in the project talks to imu_driver.h only. */

static mpu6050_t dev;
static int ready = 0;

int imu_driver_init(void)
{
    if (mpu6050_init_default(&dev) != 0)
    {
        fprintf(stderr, "imu_driver_init: mpu6050_init_default failed\n");
        return -1;
    }

    if (mpu6050_check_device(&dev) != 0)
    {
        fprintf(stderr, "imu_driver_init: mpu6050_check_device failed "
                        "(wrong chip on the bus / not responding)\n");
        return -1;
    }

    ready = 1;
    return 0;
}

int imu_driver_read_accel(float *ax, float *ay, float *az)
{
    mpu6050_accel_t accel;

    if (!ready)
        return -1;

    if (mpu6050_read_accel(&dev, &accel) != 0)
        return -1;

    *ax = accel.x;
    *ay = accel.y;
    *az = accel.z;
    return 0;
}

void imu_driver_deinit(void)
{
    if (ready)
        mpu6050_deinit(&dev);
    ready = 0;
}
