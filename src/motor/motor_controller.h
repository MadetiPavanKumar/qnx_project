/*
 * motor_controller.h - the only task that drives the motors
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

/* Initialises the DRV8833 driver, creates the Motor Controller channel
 * and starts its thread. Returns -1 on failure. */
int motor_controller_start(void);

/* Channel id (Safety and Watchdog attach to it). -1 if not started. */
int motor_controller_get_chid(void);

/* Stops the motors and releases the driver (called once at shutdown). */
void motor_controller_close(void);

#endif
