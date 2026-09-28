#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

/* Generic IMU interface. sensor_monitor.c calls ONLY these three
   functions - it never sees mpu6050.h directly. To swap chips later
   (MPU6050 -> MPU6500), write a new imu_driver_mpu6500.c implementing
   this exact header and swap which .c file is compiled; nothing else
   in the project changes. This is the same adapter pattern as
   hcsr04_driver.h / line_position.h. */

int imu_driver_init(void);

/* Returns 0 on success (values in g), -1 on failure. */
int imu_driver_read_accel(float *ax, float *ay, float *az);

void imu_driver_deinit(void);

#endif /* IMU_DRIVER_H */
