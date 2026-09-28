#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include "../common/system_types.h"

/* Tracks one ultrasonic sensor's recent behavior across ticks, so
   Safety can tell "sensor says far" apart from "sensor is lying." */
typedef struct
{
    float            last_good_cm;
    int              have_last_good;
    int              consecutive_bad;
    sensor_health_t  health;
} sensor_ctx_t;

void sensor_ctx_init(sensor_ctx_t *ctx);

/* raw_samples: SAMPLES_PER_TICK raw readings just taken from the
   HC-SR04 driver this tick (cm). A driver timeout should be passed
   in as a clearly-out-of-range value (e.g. -1 or 400+) - this
   function treats those as bad samples, not as "very far away."

   Returns the accepted distance for this tick and updates
   ctx->health as a side effect. */
float sensor_ctx_process(sensor_ctx_t *ctx, const float *raw_samples, int n);

#endif /* SENSOR_HEALTH_H */
