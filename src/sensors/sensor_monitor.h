/*
 * sensor_monitor.h - periodic sensor reader + filter (see sensor_monitor.c)
 */
#ifndef SENSOR_MONITOR_H
#define SENSOR_MONITOR_H

/* Opens both ultrasonic sensors and starts the Sensor Monitor thread.
 * Returns -1 if a sensor cannot be initialised.
 * If test_hang is non-zero the thread deliberately freezes for a few
 * seconds once (to demo the watchdog and the stale-data protection). */
int sensor_monitor_start(int test_hang);

/* Releases the sensor driver slots (called once at shutdown). */
void sensor_monitor_close(void);

#endif
