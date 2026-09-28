#ifndef GPIO_MAP_H
#define GPIO_MAP_H

/* Single source of truth for every GPIO pin assignment on the robot.
   Every driver adapter includes this instead of hard-coding numbers,
   so a rewire only ever means editing this one file. BCM numbering. */

/* Motor driver: switched from L298N to DRV8833 (phase/enable wiring,
   one shared PWM speed pin + independent per-wheel direction pins).
   l298n.c/h are still in drivers/ but are no longer wired into the
   project - kept only for reference in case of a future reversion.
   Pin numbers below match the tested drv8833-motor-driver.c exactly. */
#define GPIO_DRV8833_EN          18   /* shared PWM enable / nSLEEP */
#define GPIO_DRV8833_RIGHT_FWD   10   /* AIN1 */
#define GPIO_DRV8833_RIGHT_REV    9   /* AIN2 */
#define GPIO_DRV8833_LEFT_FWD     8   /* BIN1 */
#define GPIO_DRV8833_LEFT_REV    11   /* BIN2 */

/* HC-SR04 ultrasonic sensors */
#define GPIO_HCSR04_0_TRIG   5
#define GPIO_HCSR04_0_ECHO   6
#define GPIO_HCSR04_1_TRIG  16
#define GPIO_HCSR04_1_ECHO  19

/* Wheel encoders (F429 pulse output, one per wheel) */
#define GPIO_ENCODER_LEFT   20
#define GPIO_ENCODER_RIGHT  21

/* 5-channel IR lane-detection array - matches your tested
   ir_5ch_test.c wiring exactly. These reuse GPIO17/22/23/27, which
   were only ever reserved for L298N - safe now that L298N has been
   replaced by DRV8833 and is no longer wired into the project. */
#define GPIO_IR_CH1         17
#define GPIO_IR_CH2         27
#define GPIO_IR_CH3         22
#define GPIO_IR_CH4         23
#define GPIO_IR_CH5         24

/* MPU6050 and OLED both sit on the hardware I2C1 bus (GPIO2/3),
   distinguished by I2C address (0x68 vs 0x3C) rather than by GPIO -
   see mpu6050.h / oled.h, nothing to assign here. */

#endif /* GPIO_MAP_H */
