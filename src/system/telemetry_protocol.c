#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "telemetry_protocol.h"
#include "../common/labels.h"
#include "../common/config.h"

/* ---- Bounded append buffer - every write is checked; once it
   overflows, every subsequent append becomes a no-op and the build
   function reports -1. Never allocates. ---- */
typedef struct { char *buf; size_t cap; size_t len; int overflow; } jbuf_t;

static void jinit(jbuf_t *j, char *buf, size_t cap)
{
    j->buf = buf; j->cap = cap; j->len = 0; j->overflow = 0;
    if (cap > 0) buf[0] = '\0';
}

static void japp(jbuf_t *j, const char *fmt, ...)
{
    if (j->overflow) return;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(j->buf + j->len, j->cap - j->len, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= j->cap - j->len)
    {
        j->overflow = 1;
        return;
    }
    j->len += (size_t)n;
}

/* Minimal escaper - every string passed through this is our own
   (robot_id, module names, a decision_reason built entirely from our
   own label strings) rather than arbitrary external input, but this
   is cheap enough to always apply rather than trust that. */
static void japp_str(jbuf_t *j, const char *s)
{
    if (j->overflow) return;
    japp(j, "\"");
    for (; *s && !j->overflow; s++)
    {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\')       japp(j, "\\%c", c);
        else if (c == '\n')              japp(j, "\\n");
        else if (c == '\r')              japp(j, "\\r");
        else if (c == '\t')              japp(j, "\\t");
        else if (c < 0x20)               japp(j, "\\u%04x", c);
        else                              japp(j, "%c", c);
    }
    japp(j, "\"");
}

/* ---- Labels for enums that don't already have one in labels.h
   (safety_state_t/lane_position_t/sensor_health_t do - reused
   directly below). Kept local/static: these are protocol-layer
   presentation strings for the JSON wire format, not a general UI
   label API like labels.h. ---- */

static const char *watchdog_state_label(watchdog_state_t s)
{
    switch (s)
    {
        case WATCHDOG_HEALTHY:         return "HEALTHY";
        case WATCHDOG_MISSED_DEADLINE:  return "MISSED_DEADLINE";
        case WATCHDOG_PERSISTENT_FAULT: return "PERSISTENT_FAULT";
        default:                        return "UNKNOWN";
    }
}

static const char *recovery_state_label(recovery_state_t s)
{
    switch (s)
    {
        case RECOVERY_STATE_NONE:              return "NONE";
        case RECOVERY_STATE_SCHEDULED:          return "SCHEDULED";
        case RECOVERY_STATE_RUNNING:            return "RUNNING";
        case RECOVERY_STATE_RECOVERED:          return "RECOVERED";
        case RECOVERY_STATE_PERSISTENT_FAULT:   return "PERSISTENT_FAULT";
        default:                                return "UNKNOWN";
    }
}

static const char *recovery_result_label(recovery_result_t r)
{
    switch (r)
    {
        case RECOVERY_RESULT_SUCCESS: return "SUCCESS";
        case RECOVERY_RESULT_FAILURE: return "FAILURE";
        case RECOVERY_RESULT_NONE:    return "NONE";
        default:                      return "UNKNOWN";
    }
}

static const char *wheel_dir_label(wheel_dir_t d)
{
    switch (d)
    {
        case WHEEL_STOP:    return "STOP";
        case WHEEL_FORWARD: return "FORWARD";
        case WHEEL_REVERSE: return "REVERSE";
        default:            return "UNKNOWN";
    }
}

static const char *nav_command_label(navigation_command_t c)
{
    switch (c)
    {
        case CMD_STOP:     return "STOP";
        case CMD_FORWARD:  return "FORWARD";
        case CMD_BACKWARD: return "BACKWARD";
        case CMD_LEFT:     return "LEFT";
        case CMD_RIGHT:    return "RIGHT";
        default:           return "UNKNOWN";
    }
}

static const char *nav_health_label(navigation_health_t h)
{
    return (h == NAV_HEALTH_OK) ? "OK" : "FAULT";
}

static const char *event_type_label(telemetry_event_type_t t)
{
    switch (t)
    {
        case TELEMETRY_EVT_SAFETY_STATE_CHANGED:      return "SAFETY_STATE_CHANGED";
        case TELEMETRY_EVT_EMERGENCY_STOP:            return "EMERGENCY_STOP";
        case TELEMETRY_EVT_MANUAL_ESTOP:              return "MANUAL_ESTOP";
        case TELEMETRY_EVT_SENSOR_FAULT:              return "SENSOR_FAULT";
        case TELEMETRY_EVT_SENSOR_RECOVERY_STARTED:   return "SENSOR_RECOVERY_STARTED";
        case TELEMETRY_EVT_SENSOR_RECOVERY_SUCCESS:   return "SENSOR_RECOVERY_SUCCESS";
        case TELEMETRY_EVT_SENSOR_RECOVERY_FAILED:    return "SENSOR_RECOVERY_FAILED";
        case TELEMETRY_EVT_WATCHDOG_MISSED:           return "WATCHDOG_MISSED";
        case TELEMETRY_EVT_WATCHDOG_PERSISTENT_FAULT: return "WATCHDOG_PERSISTENT_FAULT";
        case TELEMETRY_EVT_SAFETY_DEADLINE_VIOLATION: return "SAFETY_DEADLINE_VIOLATION";
        case TELEMETRY_EVT_SENSOR_DATA_STALE:         return "SENSOR_DATA_STALE";
        case TELEMETRY_EVT_MOTOR_FAULT:               return "MOTOR_FAULT";
        case TELEMETRY_EVT_NAVIGATION_FAULT:          return "NAVIGATION_FAULT";
        case TELEMETRY_EVT_WIFI_DISCONNECTED:         return "WIFI_DISCONNECTED";
        case TELEMETRY_EVT_WIFI_RECONNECTED:          return "WIFI_RECONNECTED";
        case TELEMETRY_EVT_SUPABASE_UPLOAD_FAILED:    return "SUPABASE_UPLOAD_FAILED";
        case TELEMETRY_EVT_SUPABASE_RECONNECTED:      return "SUPABASE_RECONNECTED";
        default:                                       return "UNKNOWN";
    }
}

static const char *severity_label(telemetry_severity_t s)
{
    switch (s)
    {
        case TELEMETRY_SEV_DEBUG: return "DEBUG";
        case TELEMETRY_SEV_INFO:  return "INFO";
        case TELEMETRY_SEV_WARN:  return "WARN";
        case TELEMETRY_SEV_ERROR: return "ERROR";
        default:                   return "UNKNOWN";
    }
}

static void japp_watchdog(jbuf_t *j, const telemetry_watchdog_snapshot_t *w)
{
    if (!w->has_heartbeat)
    {
        japp(j, "{\"state\":\"NO_HEARTBEAT_YET\",\"configured_timeout_ms\":%d}",
             w->configured_timeout_ms);
        return;
    }
    japp(j, "{\"state\":");
    japp_str(j, watchdog_state_label(w->state));
    japp(j, ",\"heartbeat_age_ms\":%llu,\"consecutive_misses\":%d,"
            "\"configured_timeout_ms\":%d}",
         (unsigned long long)w->heartbeat_age_ms, w->consecutive_misses,
         w->configured_timeout_ms);
}

static void japp_recovery(jbuf_t *j, const telemetry_recovery_snapshot_t *r)
{
    japp(j, "{\"module\":");
    japp_str(j, r->module);
    japp(j, ",\"recovery_state\":");
    japp_str(j, recovery_state_label(r->recovery_state));
    japp(j, ",\"attempt_count\":%d,\"next_retry_time_ms\":%llu,"
            "\"last_recovery_result\":",
         r->attempt_count, (unsigned long long)r->next_retry_time_ms);
    japp_str(j, recovery_result_label(r->last_recovery_result));
    japp(j, ",\"failure_count\":%d}", r->failure_count);
}

static void japp_latency(jbuf_t *j, const task_latency_stats_t *s)
{
    japp(j, "{\"min_ms\":%.2f,\"max_ms\":%.2f,\"avg_ms\":%.2f,\"samples\":%d}",
         s->min_ms, s->max_ms, s->avg_ms, s->samples);
}

int telemetry_protocol_build_snapshot_json(const telemetry_snapshot_t *snap,
                                            char *buf, size_t buf_size)
{
    jbuf_t j;
    jinit(&j, buf, buf_size);

    japp(&j, "{");

    /* -- metadata (item 16) -- */
    japp(&j, "\"schema_version\":%u,\"robot_id\":", snap->meta.schema_version);
    japp_str(&j, snap->meta.robot_id);
    japp(&j, ",\"boot_id\":%u,\"sequence\":%llu,"
             "\"timestamp_monotonic_ms\":%llu,\"timestamp_wallclock\":%llu,"
             "\"wallclock_valid\":%s,\"uptime_ms\":%llu",
         snap->meta.boot_id, (unsigned long long)snap->meta.sequence_number,
         (unsigned long long)snap->meta.timestamp_monotonic_ms,
         (unsigned long long)snap->meta.timestamp_wallclock_s,
         snap->meta.wallclock_valid ? "true" : "false",
         (unsigned long long)snap->meta.uptime_ms);

    /* -- safety (item 17) - top-level scalar columns, per item 40 -- */
    japp(&j, ",\"safety_state\":");
    japp_str(&j, safety_state_label(snap->safety.state));
    japp(&j, ",\"requested_speed\":%d,\"approved_speed\":%d",
         snap->safety.requested_speed_percent, snap->safety.approved_speed_percent);
    japp(&j, ",\"safety\":{\"processing_time_ms\":%.2f,\"deadline_bound_ms\":%d,"
             "\"deadline_violated\":%s,\"snapshot_stale\":%s,"
             "\"manual_estop_active\":%s,\"decision_reason\":",
         snap->safety.safety_processing_time_ms, snap->safety.safety_deadline_bound_ms,
         snap->safety.safety_deadline_violated ? "true" : "false",
         snap->safety.snapshot_stale ? "true" : "false",
         snap->safety.manual_estop_active ? "true" : "false");
    japp_str(&j, snap->safety.decision_reason);
    japp(&j, "}");

    /* -- sensors (item 18-21), grouped into one JSONB column -- */
    japp(&j, ",\"sensors\":{\"ultrasonic\":[");
    for (int i = 0; i < 2; i++)
    {
        const telemetry_ultrasonic_snapshot_t *u = &snap->ultrasonic[i];
        japp(&j, "%s{\"distance_cm\":%.1f,\"health\":", i ? "," : "",
             (double)u->distance_cm);
        japp_str(&j, sensor_health_label(u->health));
        japp(&j, ",\"valid\":%s,\"stale\":%s,\"last_update_age_ms\":%llu,"
                 "\"fault_count\":%d,\"recovery_state\":",
             u->valid ? "true" : "false", u->stale ? "true" : "false",
             (unsigned long long)u->last_update_age_ms, u->fault_count);
        japp_str(&j, recovery_state_label(u->recovery_state));
        japp(&j, "}");
    }
    japp(&j, "],\"ir\":{\"lane_position\":");
    japp_str(&j, lane_position_label(snap->ir.lane_position));
    japp(&j, ",\"lane_health\":");
    japp_str(&j, sensor_health_label(snap->ir.lane_health));
    japp(&j, ",\"last_update_age_ms\":%llu}",
         (unsigned long long)snap->ir.last_update_age_ms);

    japp(&j, ",\"encoder\":{\"left_rpm\":%.1f,\"right_rpm\":%.1f,\"health\":",
         (double)snap->encoder.left_rpm, (double)snap->encoder.right_rpm);
    japp_str(&j, sensor_health_label(snap->encoder.health));
    japp(&j, ",\"left_direction\":");
    japp_str(&j, wheel_dir_label(snap->encoder.left_direction));
    japp(&j, ",\"right_direction\":");
    japp_str(&j, wheel_dir_label(snap->encoder.right_direction));
    japp(&j, ",\"last_update_age_ms\":%llu}",
         (unsigned long long)snap->encoder.last_update_age_ms);

    japp(&j, ",\"imu\":{\"accel_x\":%.3f,\"accel_y\":%.3f,\"accel_z\":%.3f,\"health\":",
         (double)snap->imu.accel_x, (double)snap->imu.accel_y, (double)snap->imu.accel_z);
    japp_str(&j, sensor_health_label(snap->imu.health));
    japp(&j, ",\"last_update_age_ms\":%llu}}",
         (unsigned long long)snap->imu.last_update_age_ms);

    /* -- motor (item 22) -- */
    japp(&j, ",\"motor\":{\"requested_speed\":%u,\"approved_speed\":%u,"
             "\"left_direction\":",
         snap->motor.requested_motor_speed, snap->motor.approved_motor_speed);
    japp_str(&j, wheel_dir_label(snap->motor.left_direction));
    japp(&j, ",\"right_direction\":");
    japp_str(&j, wheel_dir_label(snap->motor.right_direction));
    japp(&j, ",\"emergency_stop\":%s,\"command_sequence\":%llu,"
             "\"command_timestamp_ms\":%llu,\"controller_health\":",
         snap->motor.emergency_stop ? "true" : "false",
         (unsigned long long)snap->motor.command_sequence,
         (unsigned long long)snap->motor.command_timestamp_ms);
    japp_str(&j, watchdog_state_label(snap->motor.controller_health));
    japp(&j, "}");

    /* -- navigation (item 23) -- */
    japp(&j, ",\"navigation\":{\"command\":");
    japp_str(&j, nav_command_label(snap->navigation.command));
    japp(&j, ",\"requested_speed\":%d,\"health\":",
         snap->navigation.requested_speed_percent);
    japp_str(&j, nav_health_label(snap->navigation.health));
    japp(&j, "}");

    /* -- watchdog (item 24) -- */
    japp(&j, ",\"watchdog\":{\"navigation\":");
    japp_watchdog(&j, &snap->watchdog[MODULE_NAVIGATION]);
    japp(&j, ",\"safety\":");
    japp_watchdog(&j, &snap->watchdog[MODULE_SAFETY]);
    japp(&j, ",\"sensor_monitor\":");
    japp_watchdog(&j, &snap->watchdog[MODULE_SENSOR_MONITOR]);
    japp(&j, ",\"motor_controller\":");
    japp_watchdog(&j, &snap->watchdog[MODULE_MOTOR_CONTROLLER]);
    japp(&j, "}");

    /* -- recovery (item 25) -- */
    japp(&j, ",\"recovery\":[");
    for (int i = 0; i < TELEMETRY_RECOVERY_ENTRIES; i++)
    {
        japp(&j, "%s", i ? "," : "");
        japp_recovery(&j, &snap->recovery[i]);
    }
    japp(&j, "]");

    /* -- latency (item 26) -- */
    japp(&j, ",\"latency\":{\"safety\":{\"cycle\":");
    japp_latency(&j, &snap->safety_latency.cycle);
    japp(&j, ",\"processing\":");
    japp_latency(&j, &snap->safety_latency.processing);
    japp(&j, "},\"sensor_monitor\":{\"cycle\":");
    japp_latency(&j, &snap->sensor_latency.cycle);
    japp(&j, ",\"ultrasonic\":");
    japp_latency(&j, &snap->sensor_latency.ultrasonic);
    japp(&j, ",\"ir\":");
    japp_latency(&j, &snap->sensor_latency.ir);
    japp(&j, ",\"encoder\":");
    japp_latency(&j, &snap->sensor_latency.encoder);
    japp(&j, ",\"imu\":");
    japp_latency(&j, &snap->sensor_latency.imu);
    japp(&j, "},\"navigation\":{\"period\":");
    japp_latency(&j, &snap->navigation_latency.period);
    japp(&j, ",\"execution\":");
    japp_latency(&j, &snap->navigation_latency.execution);
    japp(&j, "},\"motor\":{\"period\":");
    japp_latency(&j, &snap->motor_latency.period);
    japp(&j, ",\"execution\":");
    japp_latency(&j, &snap->motor_latency.execution);
    japp(&j, "},\"display\":{\"oled_update\":");
    japp_latency(&j, &snap->display_latency.oled_update);
    japp(&j, "}}");

    japp(&j, "}");

    return j.overflow ? -1 : (int)j.len;
}

int telemetry_protocol_build_events_json(const telemetry_event_record_t *events,
                                          int count, char *buf, size_t buf_size)
{
    jbuf_t j;
    jinit(&j, buf, buf_size);

    japp(&j, "[");
    for (int i = 0; i < count; i++)
    {
        const telemetry_event_record_t *e = &events[i];
        japp(&j, "%s{\"robot_id\":", i ? "," : "");
        japp_str(&j, ROBOT_ID);
        japp(&j, ",\"timestamp_monotonic_ms\":%llu,\"event_type\":",
             (unsigned long long)e->time_ms);
        japp_str(&j, event_type_label(e->type));
        japp(&j, ",\"severity\":");
        japp_str(&j, severity_label(e->severity));
        japp(&j, ",\"module\":");
        japp_str(&j, e->module);
        japp(&j, ",\"message\":");
        japp_str(&j, e->message);
        japp(&j, "}");
    }
    japp(&j, "]");

    return j.overflow ? -1 : (int)j.len;
}
