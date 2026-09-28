#ifndef LINE_POSITION_H
#define LINE_POSITION_H

#include "../common/system_types.h"

/* Thin adapter over your real ir_array_driver.h, matching the
   hcsr04_driver.h pattern: translates the real driver's types/API
   into the project-owned lane_position_t / sensor_health_t so
   safety_supervisor.c and messages.h never need to know anything
   about IR_Position, IR_Config, or IRArray_* directly. */

/* Call once, after the channel-ready signal (see safety_supervisor.c
   for why hardware init happens after, not before, that signal).
   Returns 0 on success, -1 on failure. */
int line_position_init(void);

/* Reads all 5 channels for this tick and reports:
   - out_position: CENTER/LEFT/.../LOST, or LANE_UNKNOWN if this
     tick's read failed or the pattern was ambiguous.
   - out_health: OK if the read succeeded cleanly, DEGRADED if the
     driver is accumulating failures but not yet unhealthy, FAULT if
     the driver has declared itself unhealthy (IRArray_IsHealthy()
     false) - mirrors sensor_health_t's meaning for the ultrasonic
     sensors, so Safety/the display can treat all sensor health
     uniformly. */
void line_position_read(lane_position_t *out_position, sensor_health_t *out_health);

void line_position_deinit(void);

#endif /* LINE_POSITION_H */
