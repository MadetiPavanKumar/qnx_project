#ifndef TELEMETRY_EVENTS_H
#define TELEMETRY_EVENTS_H

/* Deliberately its own tiny header, separate from telemetry_service.h
   - any task that wants to publish a transition event only needs
   this (an enum + telemetry_publish_event()'s declaration, pulled in
   via telemetry_service.h), not the connection-state-machine/
   statistics machinery that comes with the rest of that file. */

/* item 28 - transition events only, never one per control cycle. */
typedef enum
{
    TELEMETRY_EVT_SAFETY_STATE_CHANGED = 0,
    TELEMETRY_EVT_EMERGENCY_STOP,
    TELEMETRY_EVT_MANUAL_ESTOP,
    TELEMETRY_EVT_SENSOR_FAULT,
    TELEMETRY_EVT_SENSOR_RECOVERY_STARTED,
    TELEMETRY_EVT_SENSOR_RECOVERY_SUCCESS,
    TELEMETRY_EVT_SENSOR_RECOVERY_FAILED,
    TELEMETRY_EVT_WATCHDOG_MISSED,
    TELEMETRY_EVT_WATCHDOG_PERSISTENT_FAULT,
    TELEMETRY_EVT_SAFETY_DEADLINE_VIOLATION,
    TELEMETRY_EVT_SENSOR_DATA_STALE,
    TELEMETRY_EVT_MOTOR_FAULT,
    TELEMETRY_EVT_NAVIGATION_FAULT,
    /* Not yet fired by anything - there is no transport in this
       phase to be dis/reconnected. Reserved so Phase 4's transport
       code can use these without another header change. */
    TELEMETRY_EVT_WIFI_DISCONNECTED,
    TELEMETRY_EVT_WIFI_RECONNECTED,
    TELEMETRY_EVT_SUPABASE_UPLOAD_FAILED,
    TELEMETRY_EVT_SUPABASE_RECONNECTED,
    TELEMETRY_EVT_COUNT
} telemetry_event_type_t;

/* Mirrors log_level_t's ordering (common/logger.h) on purpose - same
   "DEBUG is disposable first" logic applies to the event queue's
   overflow policy (item 29) as to the logger's. */
typedef enum
{
    TELEMETRY_SEV_DEBUG = 0,
    TELEMETRY_SEV_INFO  = 1,
    TELEMETRY_SEV_WARN  = 2,
    TELEMETRY_SEV_ERROR = 3
} telemetry_severity_t;

#endif /* TELEMETRY_EVENTS_H */
