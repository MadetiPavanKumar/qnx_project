#ifndef LABELS_H
#define LABELS_H

#include "system_types.h"

/* Shared human-readable labels - used by both the OLED and the CLI so
   the two never drift out of sync with each other. */
const char *safety_state_label(safety_state_t s);
const char *lane_position_label(lane_position_t p);
const char *sensor_health_label(sensor_health_t h);

#endif /* LABELS_H */
