#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdarg.h>
#include "../common/system_types.h"

/* Shared, mutex-protected system state - written by Safety every
   tick, read by the CLI (and the OLED, via the same snapshot) so
   there is exactly one place that knows "what's true right now."
   This is also the backing store for the spec's required Safety
   Events / Override Latency / Health Status logging. */

typedef struct
{
    uint64_t         last_update_ms;

    safety_state_t   state;
    int              approved_speed_percent;
    int              requested_speed_percent;
    float            processing_time_ms;
    int              deadline_bound_ms;
    int              snapshot_stale;   /* sensor snapshot was stale THIS
                                           tick - see SENSOR_SNAPSHOT_STALE_MS
                                           in config.h. Added for the
                                           telemetry service's safety
                                           snapshot (item 17); Safety already
                                           computed this locally every tick,
                                           this just publishes it alongside
                                           everything else here instead of
                                           discarding it after the tick. */

    float            ultrasonic_cm[2];
    sensor_health_t  ultrasonic_health[2];

    lane_position_t  lane_position;
    sensor_health_t  lane_health;

    float            left_rpm;
    float            right_rpm;
    sensor_health_t  encoder_health;

    float            accel_x, accel_y, accel_z;
    sensor_health_t  imu_health;

    int              manual_estop_active;
} telemetry_t;

void telemetry_update(const telemetry_t *snapshot);
void telemetry_get(telemetry_t *out);

#define EVENT_MESSAGE_MAX 96

typedef struct
{
    uint64_t time_ms;
    char     message[EVENT_MESSAGE_MAX];
} event_entry_t;

/* printf-style. Safe to call from any thread. */
void event_log_add(const char *fmt, ...);

/* Copies up to max_count of the most recent events into out, newest
   first. Returns how many were actually copied. */
int event_log_get_recent(event_entry_t *out, int max_count);

#endif /* TELEMETRY_H */
