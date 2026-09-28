#ifndef TELEMETRY_SERVICE_H
#define TELEMETRY_SERVICE_H

#include <stdint.h>

#include "telemetry.h"
#include "telemetry_events.h"
#include "recovery_manager.h"
#include "../common/system_types.h"
#include "../common/task_stats.h"
#include "../common/watchdog.h"
#include "../safety/safety_supervisor.h"
#include "../sensors/sensor_monitor.h"
#include "../navigation/navigation.h"
#include "../motor/motor_controller.h"
#include "../display/display_task.h"

/* ============================================================
 * TELEMETRY SERVICE - the cloud-facing OBSERVER layer
 * ============================================================
 *
 * Built entirely ON TOP OF the existing telemetry.c (Safety's
 * synchronous safety/sensor snapshot + the local event ring) and the
 * read-only getters every task now exposes (safety_supervisor_
 * get_stats(), sensor_monitor_get_stats(), navigation_get_stats()/
 * get_status(), motor_controller_get_stats()/get_status(),
 * display_get_stats(), recovery_manager_get_watchdog_status()/
 * get_recovery_status()). This file does not duplicate any of their
 * state - it only reads it, on its own low-priority thread, roughly
 * once a second (TELEMETRY_SNAPSHOT_INTERVAL_MS in config.h), and
 * assembles ONE combined snapshot struct from all of it.
 *
 * NETWORKING lives in system/telemetry_transport.c (OpenSSL over a
 * persistent TCP socket) and system/telemetry_protocol.c (JSON
 * serialization) - this file orchestrates them (run_upload_cycle()
 * in the .c file) but has no socket/TLS code of its own, so a
 * different transport library could replace telemetry_transport.c
 * without this file changing:
 *
 *   - telemetry_publish_event()      - bounded async event queue (item 29)
 *   - the connection state machine   - item 39, actually driven by
 *                                       upload attempt results now
 *                                       (see report_upload_result()
 *                                       in the .c file)
 *   - telemetry_get_connection_status() / telemetry_get_statistics() (item 12/27)
 *   - telemetry_service_get_snapshot() - the full assembled snapshot
 *     (items 15-26), serialized and uploaded once per cycle by
 *     run_upload_cycle()
 *
 * Every one of the getters this file calls is already documented as
 * "safe to call from any thread, never blocks" - so this whole
 * service (transport included) can never itself block or slow down
 * any real-time task (item 13/33/34): it only ever reads their
 * published state, never their internals, and every network wait it
 * performs is bounded and confined to its own low-priority thread.
 */

/* ---- Bounded event queue (item 28/29) -------------------------- */

/* Safe to call from ANY thread, including real-time tasks - this is
   the ONLY telemetry entry point a real-time task should call
   directly besides telemetry_update()/event_log_add() (telemetry.h).
   Formats into a stack buffer, then does a bounded, O(1) push into a
   fixed-size ring (TELEMETRY_EVENT_QUEUE_CAPACITY, config.h) and
   returns immediately - never allocates, never blocks, never does
   I/O. Overflow policy mirrors the logger's (see logger.h): the
   single oldest queued entry is evicted if it's DEBUG-severity to
   make room; otherwise the incoming entry itself is dropped. Either
   way, telemetry_get_statistics()'s dropped_events increments.
   Deliberately for TRANSITIONS only (item 28) - do not call this
   once per control cycle. */
void telemetry_publish_event(telemetry_event_type_t type,
                              telemetry_severity_t severity,
                              const char *module,
                              const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

#define TELEMETRY_EVENT_MESSAGE_MAX 96
#define TELEMETRY_LABEL_MAX         24

typedef struct
{
    uint64_t                time_ms;   /* CLOCK_MONOTONIC */
    telemetry_event_type_t  type;
    telemetry_severity_t    severity;
    char                    module[TELEMETRY_LABEL_MAX];
    char                    message[TELEMETRY_EVENT_MESSAGE_MAX];
} telemetry_event_record_t;

/* Pops up to `max` queued events, OLDEST first, REMOVING them from
   the queue - only the transport layer (system/telemetry_transport.c
   via telemetry_service.c) should call this; popping without a
   successful upload means the event is gone from telemetry's queue
   for good (the local, authoritative event ring in telemetry.h is
   unaffected either way - see the file header comment above).
   Returns how many were actually popped (0 if the queue was empty). */
int telemetry_drain_events(telemetry_event_record_t *out, int max);

/* ---- Connection state machine (item 39) ------------------------- */

typedef enum
{
    TELEMETRY_CONN_OFFLINE = 0,   /* no transport configured (env vars unset),
                                      or no upload attempted yet */
    TELEMETRY_CONN_CONNECTING,
    TELEMETRY_CONN_ONLINE,
    TELEMETRY_CONN_DEGRADED,      /* connected, but recent uploads have failed */
    TELEMETRY_CONN_BACKOFF        /* waiting out a retry delay after failure(s) */
} telemetry_conn_state_t;

typedef struct
{
    telemetry_conn_state_t state;
    uint64_t  last_successful_upload_ms; /* CLOCK_MONOTONIC; 0 = never */
    uint64_t  last_failed_upload_ms;     /* CLOCK_MONOTONIC; 0 = never */
    int       consecutive_failures;
    uint64_t  total_failures;
    uint64_t  total_successful_uploads;
    int       queued_events;             /* current event-queue depth */
    uint64_t  dropped_events;
} telemetry_conn_status_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy. */
void telemetry_get_connection_status(telemetry_conn_status_t *out);

/* ---- Performance/queue statistics (item 27) ---------------------- */

typedef struct
{
    float     snapshot_build_ms;         /* time to assemble one snapshot */
    float     serialization_ms;          /* time to build the JSON body */
    float     network_connect_ms;        /* 0 if the persistent connection
                                              was reused, no new connect */
    float     network_send_ms;
    float     server_response_ms;
    float     total_upload_ms;
    float     telemetry_thread_jitter_ms; /* tick-to-tick period jitter, same
                                              pattern as every other task's
                                              period stat */
    int       queue_depth;               /* == connection_status.queued_events */
    uint64_t  dropped_events;
    uint64_t  dropped_logs;              /* from logger_get_dropped_count() */
} telemetry_service_stats_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy, refreshed once per assembled snapshot
   (~TELEMETRY_SNAPSHOT_INTERVAL_MS). */
void telemetry_get_statistics(telemetry_service_stats_t *out);

/* ---- The assembled snapshot (items 15-26) ------------------------ */

#define TELEMETRY_ROBOT_ID_MAX     32
#define TELEMETRY_REASON_MAX       96

typedef struct
{
    uint32_t schema_version;
    char     robot_id[TELEMETRY_ROBOT_ID_MAX];
    uint32_t boot_id;             /* random-ish value picked at process start -
                                      lets a dashboard tell two runs of the
                                      same robot_id apart after a restart */
    uint64_t sequence_number;     /* increments once per snapshot - item 45 */
    uint64_t timestamp_monotonic_ms;
    uint64_t timestamp_wallclock_s; /* seconds since epoch; see wallclock_valid */
    int      wallclock_valid;       /* 0 if CLOCK_REALTIME looks unset (pre-NTP
                                        on a fresh boot with no RTC) - item 16:
                                        "never use cloud time for safety
                                        decisions", so this is metadata only,
                                        never fed back into any decision */
    uint64_t uptime_ms;
} telemetry_meta_t;

typedef struct
{
    safety_state_t state;
    int   requested_speed_percent;
    int   approved_speed_percent;
    float safety_processing_time_ms;
    int   safety_deadline_bound_ms;
    int   safety_deadline_violated;   /* most recent tick */
    int   snapshot_stale;             /* sensor snapshot was stale THIS tick */
    int   manual_estop_active;
    char  decision_reason[TELEMETRY_REASON_MAX]; /* derived, human-readable -
                                                      formatted from the fields
                                                      above at snapshot time,
                                                      not a separately stored
                                                      value */
} telemetry_safety_snapshot_t;

typedef struct
{
    float             distance_cm;
    sensor_health_t   health;
    int               valid;             /* health != SENSOR_FAULT */
    int               stale;             /* mirrors snapshot_stale - the
                                             project only tracks staleness for
                                             the whole sensor snapshot, not
                                             per-channel; not fabricated finer
                                             than that */
    uint64_t          last_update_age_ms;
    int                fault_count;       /* from recovery_manager - shared
                                              across both ultrasonic channels,
                                              see note below */
    recovery_state_t  recovery_state;    /* also shared across both channels:
                                              sensor_monitor.c reports BOTH
                                              HC-SR04s under the single
                                              recovery-manager name "hcsr04",
                                              so there is no real per-channel
                                              recovery history to report
                                              separately */
} telemetry_ultrasonic_snapshot_t;

typedef struct
{
    lane_position_t  lane_position;
    sensor_health_t  lane_health;
    /* raw_channel_state (item 19) is NOT included: line_position.c's
       driver only returns the aggregated lane_position, not the 5
       raw IR channel bits - exposing that would mean adding a new
       read-only getter to the IR driver itself, which this phase
       deliberately did not touch. navigation_interpretation is
       covered by lane_position/lane_health above - there is no
       separate navigation-side interpretation of the IR data beyond
       what Safety already consumes. */
    uint64_t last_update_age_ms;
} telemetry_ir_snapshot_t;

typedef struct
{
    float           left_rpm;
    float           right_rpm;
    sensor_health_t health;
    wheel_dir_t     left_direction;   /* derived from sign of left_rpm  */
    wheel_dir_t     right_direction;  /* derived from sign of right_rpm */
    uint64_t        last_update_age_ms;
    /* pulse_counts (item 20) is NOT included: encoder_driver.c only
       exposes RPM (encoder_driver_read_rpm()), not raw pulse counts -
       not fabricated. */
} telemetry_encoder_snapshot_t;

typedef struct
{
    float           accel_x, accel_y, accel_z;
    sensor_health_t health;
    uint64_t        last_update_age_ms;
    /* orientation (item 21) is NOT included: nothing in this project
       computes an orientation/quaternion from the raw accel - not
       fabricated. */
} telemetry_imu_snapshot_t;

typedef struct
{
    unsigned int      requested_motor_speed;
    unsigned int      approved_motor_speed;
    wheel_dir_t       left_direction;
    wheel_dir_t       right_direction;
    int               emergency_stop;
    uint64_t          command_sequence;
    uint64_t          command_timestamp_ms;
    watchdog_state_t  controller_health; /* from Recovery Manager's watchdog
                                             tracking for MODULE_MOTOR_CONTROLLER -
                                             reusing the existing enum rather
                                             than inventing a parallel one */
} telemetry_motor_snapshot_t;

typedef struct
{
    navigation_command_t command;
    int                  requested_speed_percent;
    navigation_health_t  health;
    /* navigation_state (item 23) is NOT included: decide_intent() in
       navigation.c has no real state machine yet (always CMD_FORWARD) -
       not fabricated. */
} telemetry_navigation_snapshot_t;

typedef struct
{
    int               has_heartbeat;   /* 0 -> NO_HEARTBEAT_YET, rest of this
                                           struct is meaningless */
    watchdog_state_t  state;
    uint64_t          heartbeat_age_ms;
    int               consecutive_misses;
    int               configured_timeout_ms;
} telemetry_watchdog_snapshot_t;

/* One entry per name Recovery Manager has ever tracked - see item 25.
   `module` mirrors the exact name passed to
   recovery_manager_report_fault() (e.g. "hcsr04", "ir_array"). */
typedef struct
{
    char               module[TELEMETRY_LABEL_MAX];
    recovery_state_t   recovery_state;
    int                attempt_count;
    uint64_t           next_retry_time_ms;
    recovery_result_t  last_recovery_result;
    int                failure_count;
} telemetry_recovery_snapshot_t;

#define TELEMETRY_RECOVERY_ENTRIES 4 /* hcsr04, ir_array, encoder, imu */

typedef struct
{
    telemetry_meta_t                 meta;
    telemetry_safety_snapshot_t      safety;
    telemetry_ultrasonic_snapshot_t  ultrasonic[2];
    telemetry_ir_snapshot_t          ir;
    telemetry_encoder_snapshot_t     encoder;
    telemetry_imu_snapshot_t         imu;
    telemetry_motor_snapshot_t       motor;
    telemetry_navigation_snapshot_t  navigation;

    /* Indexed by module_id_t (watchdog.h) - MODULE_COUNT entries. */
    telemetry_watchdog_snapshot_t    watchdog[MODULE_COUNT];

    telemetry_recovery_snapshot_t    recovery[TELEMETRY_RECOVERY_ENTRIES];

    /* item 26 - one latency snapshot per task, straight from each
       task's own *_get_stats() getter. */
    safety_stats_t                  safety_latency;
    sensor_monitor_stats_t          sensor_latency;
    navigation_stats_t              navigation_latency;
    motor_controller_stats_t        motor_latency;
    display_stats_t                 display_latency;
} telemetry_snapshot_t;

/* Starts the Telemetry Service thread - see main.c for where this
   fits in startup order (no dependency on any other task's startup
   order; it only ever reads their public getters). Builds a complete
   telemetry_snapshot_t roughly every TELEMETRY_SNAPSHOT_INTERVAL_MS
   and publishes it for telemetry_service_get_snapshot() below - no
   network I/O happens in this phase (see the file header comment). */
void telemetry_service_start(void);

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy of the most recently assembled snapshot.
   meta.sequence_number == 0 if no snapshot has been assembled yet
   (right after startup). */
void telemetry_service_get_snapshot(telemetry_snapshot_t *out);

#endif /* TELEMETRY_SERVICE_H */
