#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

/* Thin adapter over your real f429_encoder.h driver for the two
   wheel encoders (left/right). NOT wired into safety_supervisor.c's
   decision logic yet - that's the next step, once you confirm your
   wheel diameter so RPM can be turned into real cm/s. */

int encoder_driver_init(void);

/* Blocks for the configured measurement window (see encoder_driver.c)
   while it counts pulses, then returns each wheel's RPM. Call this
   once per tick, same as the ultrasonic reads. */
void encoder_driver_read_rpm(float *left_rpm, float *right_rpm);

void encoder_driver_deinit(void);

#endif /* ENCODER_DRIVER_H */
