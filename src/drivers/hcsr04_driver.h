#ifndef HCSR04_DRIVER_H
#define HCSR04_DRIVER_H

/* Thin adapter over your real hc_sr04.h "simple API"
   (hc_sr04_begin / read_distance / hc_sr04_end), matching the
   sensor_index-based interface sensor_health.c / safety_supervisor.c
   already expect. */

/* Call once, before the safety loop starts reading. Returns 0 on
   success, -1 if either sensor failed to initialize. */
int hcsr04_driver_init(void);

/* sensor_index: 0 or 1. Returns distance in cm, or -1.0f on any
   failure (no echo, timeout, out of range) - sensor_health.c already
   treats anything outside 2-400cm as a bad sample, not "far away." */
float hcsr04_read_raw_cm(int sensor_index);

void hcsr04_driver_deinit(void);

#endif /* HCSR04_DRIVER_H */
