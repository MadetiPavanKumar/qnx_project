#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <stdatomic.h>

#include "telemetry_service.h"
#include "telemetry_transport.h"
#include "telemetry_protocol.h"
#include "../common/config.h"
#include "../common/logger.h"
#include "../common/labels.h"

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

/* ================================================================
 * Bounded event queue - item 28/29. Same shape/policy as logger.c's
 * queue (see the comment there); kept as an independent
 * implementation rather than a shared template because the record
 * type differs (module name + event type, not just a level) and
 * because event delivery is going to grow its own concerns in Phase 4
 * (marking entries "already uploaded" etc.) that have nothing to do
 * with the logger.
 * ================================================================ */

typedef telemetry_event_record_t event_record_t; /* local alias, unchanged below */

static event_record_t   evt_queue[TELEMETRY_EVENT_QUEUE_CAPACITY];
static int               evt_head  = 0;
static int               evt_tail  = 0;
static int               evt_count = 0;
static pthread_mutex_t   evt_lock  = PTHREAD_MUTEX_INITIALIZER;

static atomic_uint_least64_t dropped_events_total = 0;

void telemetry_publish_event(telemetry_event_type_t type,
                              telemetry_severity_t severity,
                              const char *module,
                              const char *fmt, ...)
{
    char msg[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap); /* formatting OUTSIDE the lock */
    va_end(ap);

    pthread_mutex_lock(&evt_lock);

    if (evt_count == TELEMETRY_EVENT_QUEUE_CAPACITY)
    {
        if (evt_queue[evt_head].severity == TELEMETRY_SEV_DEBUG)
        {
            evt_head = (evt_head + 1) % TELEMETRY_EVENT_QUEUE_CAPACITY;
            evt_count--;
            atomic_fetch_add(&dropped_events_total, 1);
        }
        else
        {
            atomic_fetch_add(&dropped_events_total, 1);
            pthread_mutex_unlock(&evt_lock);
            return;
        }
    }

    event_record_t *slot = &evt_queue[evt_tail];
    slot->time_ms  = now_ms();
    slot->type     = type;
    slot->severity = severity;
    snprintf(slot->module, sizeof(slot->module), "%s", module);
    snprintf(slot->message, sizeof(slot->message), "%s", msg);

    evt_tail = (evt_tail + 1) % TELEMETRY_EVENT_QUEUE_CAPACITY;
    evt_count++;

    pthread_mutex_unlock(&evt_lock);
}

/* Depth-only peek, used by the status/statistics getters below. */
static int evt_queue_depth(void)
{
    pthread_mutex_lock(&evt_lock);
    int n = evt_count;
    pthread_mutex_unlock(&evt_lock);
    return n;
}

int telemetry_drain_events(telemetry_event_record_t *out, int max)
{
    pthread_mutex_lock(&evt_lock);

    int n = evt_count < max ? evt_count : max;
    for (int i = 0; i < n; i++)
    {
        out[i] = evt_queue[evt_head];
        evt_head = (evt_head + 1) % TELEMETRY_EVENT_QUEUE_CAPACITY;
        evt_count--;
    }

    pthread_mutex_unlock(&evt_lock);
    return n;
}

/* ================================================================
 * Connection state machine - item 39. Driven entirely from
 * run_upload_cycle() below (called once per service-thread cycle) -
 * nothing outside this file ever calls into the state machine, since
 * this file is also the only caller of telemetry_transport.c.
 * ================================================================ */

static pthread_mutex_t          conn_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_conn_status_t  conn_status = { .state = TELEMETRY_CONN_OFFLINE };
static uint32_t                 backoff_ms = TELEMETRY_BACKOFF_INITIAL_MS;
static uint64_t                 backoff_until_ms = 0;

void telemetry_get_connection_status(telemetry_conn_status_t *out)
{
    pthread_mutex_lock(&conn_lock);
    *out = conn_status;
    out->queued_events = evt_queue_depth();
    out->dropped_events = atomic_load(&dropped_events_total);
    pthread_mutex_unlock(&conn_lock);
}

/* Called once per upload ATTEMPT (a snapshot post, or an event-batch
   post) with whether it succeeded. Transition rule (item 39,
   collapsed to something an attempt-by-attempt caller can drive):
 *   success            -> ONLINE, consecutive_failures reset to 0,
 *                          backoff_ms reset to its initial value.
 *   1st failure in a row -> DEGRADED (still "connected" in spirit,
 *                          just had one bad upload - don't declare
 *                          the whole link down over a single blip).
 *   2nd+ failure in a row -> BACKOFF, with backoff_until_ms pushed out
 *                          by the current backoff_ms, which then
 *                          doubles (capped at TELEMETRY_BACKOFF_MAX_MS)
 *                          for next time (item 36's bounded
 *                          exponential backoff). */
static void report_upload_result(int success)
{
    pthread_mutex_lock(&conn_lock);

    uint64_t t = now_ms();
    if (success)
    {
        conn_status.state = TELEMETRY_CONN_ONLINE;
        conn_status.last_successful_upload_ms = t;
        conn_status.consecutive_failures = 0;
        conn_status.total_successful_uploads++;
        backoff_ms = TELEMETRY_BACKOFF_INITIAL_MS;
    }
    else
    {
        conn_status.last_failed_upload_ms = t;
        conn_status.consecutive_failures++;
        conn_status.total_failures++;

        if (conn_status.consecutive_failures == 1)
        {
            conn_status.state = TELEMETRY_CONN_DEGRADED;
        }
        else
        {
            conn_status.state = TELEMETRY_CONN_BACKOFF;
            backoff_until_ms = t + backoff_ms;
            if (backoff_ms < TELEMETRY_BACKOFF_MAX_MS)
            {
                backoff_ms *= 2;
                if (backoff_ms > TELEMETRY_BACKOFF_MAX_MS) backoff_ms = TELEMETRY_BACKOFF_MAX_MS;
            }
        }
    }

    pthread_mutex_unlock(&conn_lock);
}

/* Whether this cycle should attempt an upload at all - false while a
   BACKOFF delay hasn't elapsed yet, or if no transport is configured
   (item 37: no config -> stay OFFLINE forever, never spend a cycle
   even trying). */
static int should_attempt_upload(void)
{
    if (!telemetry_transport_is_configured()) return 0;

    pthread_mutex_lock(&conn_lock);
    int due = (conn_status.state != TELEMETRY_CONN_BACKOFF) || (now_ms() >= backoff_until_ms);
    if (due && conn_status.state == TELEMETRY_CONN_OFFLINE)
    {
        conn_status.state = TELEMETRY_CONN_CONNECTING; /* first-ever attempt */
    }
    pthread_mutex_unlock(&conn_lock);
    return due;
}

/* ================================================================
 * Performance/queue statistics - item 27.
 * ================================================================ */

static pthread_mutex_t             stats_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_service_stats_t   published_stats;

void telemetry_get_statistics(telemetry_service_stats_t *out)
{
    pthread_mutex_lock(&stats_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_lock);
    out->queue_depth    = evt_queue_depth();
    out->dropped_events = atomic_load(&dropped_events_total);
    out->dropped_logs   = logger_get_dropped_count();
}

/* ================================================================
 * Snapshot assembly - items 15-26.
 * ================================================================ */

static pthread_mutex_t       snapshot_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_snapshot_t  published_snapshot; /* zero-initialized:
                                                      sequence_number starts
                                                      at 0, meaning "no
                                                      snapshot yet" until the
                                                      service thread's first
                                                      cycle */

void telemetry_service_get_snapshot(telemetry_snapshot_t *out)
{
    pthread_mutex_lock(&snapshot_lock);
    *out = published_snapshot;
    pthread_mutex_unlock(&snapshot_lock);
}

static int watchdog_timeout_for(module_id_t m)
{
    switch (m)
    {
        case MODULE_NAVIGATION:       return NAVIGATION_HEARTBEAT_TIMEOUT_MS;
        case MODULE_SAFETY:           return SAFETY_HEARTBEAT_TIMEOUT_MS;
        case MODULE_SENSOR_MONITOR:   return SENSOR_MONITOR_HEARTBEAT_TIMEOUT_MS;
        case MODULE_MOTOR_CONTROLLER: return MOTOR_CONTROLLER_HEARTBEAT_TIMEOUT_MS;
        default:                      return HEARTBEAT_TIMEOUT_MS;
    }
}

static wheel_dir_t dir_from_rpm(float rpm)
{
    if (rpm > 1.0f)  return WHEEL_FORWARD;
    if (rpm < -1.0f) return WHEEL_REVERSE;
    return WHEEL_STOP;
}

/* Short, human-readable summary of WHY Safety is in its current
   state - derived entirely from fields already in `t`/`tel` at
   snapshot time (item 17's decision_reason). Not a separately
   tracked value - just those fields formatted into one string so a
   dashboard doesn't have to re-derive the same logic client-side. */
static void build_decision_reason(char *out, size_t out_size,
                                   const telemetry_t *t, int deadline_violated)
{
    if (t->manual_estop_active)
        snprintf(out, out_size, "Manual E-STOP active");
    else if (t->snapshot_stale)
        snprintf(out, out_size, "Sensor snapshot stale");
    else if (deadline_violated)
        snprintf(out, out_size, "Safety deadline violated this tick");
    else
        snprintf(out, out_size, "%s", safety_state_label(t->state));
}

static void fill_watchdog(telemetry_watchdog_snapshot_t *out, module_id_t m)
{
    watchdog_status_t ws;
    if (recovery_manager_get_watchdog_status(m, &ws) == 0)
    {
        out->has_heartbeat       = 1;
        out->state                = ws.state;
        out->heartbeat_age_ms     = ws.age_ms;
        out->consecutive_misses   = ws.consecutive_misses;
    }
    else
    {
        out->has_heartbeat     = 0;
        out->state              = WATCHDOG_HEALTHY; /* meaningless - has_heartbeat is 0 */
        out->heartbeat_age_ms    = 0;
        out->consecutive_misses  = 0;
    }
    out->configured_timeout_ms = watchdog_timeout_for(m);
}

static void fill_recovery(telemetry_recovery_snapshot_t *out, const char *name)
{
    recovery_status_t rs;
    snprintf(out->module, sizeof(out->module), "%s", name);

    if (recovery_manager_get_recovery_status(name, &rs) == 0)
    {
        out->recovery_state      = rs.state;
        out->attempt_count        = rs.attempt_count;
        out->next_retry_time_ms   = rs.next_retry_ms;
        out->last_recovery_result = rs.last_result;
        out->failure_count        = rs.failure_count;
    }
    else
    {
        out->recovery_state      = RECOVERY_STATE_NONE;
        out->attempt_count        = 0;
        out->next_retry_time_ms   = 0;
        out->last_recovery_result = RECOVERY_RESULT_NONE;
        out->failure_count        = 0;
    }
}

/* Builds one complete snapshot into *out. Pure read-only calls into
   every getter added in Phase 2 plus telemetry_get() - nothing here
   re-reads a sensor, re-decides anything, or touches hardware. Takes
   however long ~15 short mutex-protected struct copies takes
   (microseconds) - timed by the caller for snapshot_build_ms. */
static void assemble_snapshot(telemetry_snapshot_t *out, uint64_t boot_time_ms,
                               uint32_t boot_id, uint64_t sequence)
{
    memset(out, 0, sizeof(*out));

    telemetry_t t;
    telemetry_get(&t);

    struct timespec wall;
    clock_gettime(CLOCK_REALTIME, &wall);

    /* ---- metadata (item 16) ---- */
    out->meta.schema_version         = 1;
    snprintf(out->meta.robot_id, sizeof(out->meta.robot_id), "%s", ROBOT_ID);
    out->meta.boot_id                = boot_id;
    out->meta.sequence_number        = sequence;
    out->meta.timestamp_monotonic_ms = now_ms();
    out->meta.timestamp_wallclock_s  = (uint64_t)wall.tv_sec;
    /* A wallclock reading before ~2020 means no RTC/NTP has set the
       clock yet (common on a fresh embedded boot) - flagged, never
       silently treated as real. Never fed into any safety decision
       either way (item 16). */
    out->meta.wallclock_valid        = (wall.tv_sec > 1577836800);
    out->meta.uptime_ms              = now_ms() - boot_time_ms;

    /* ---- latency (item 26) - safety_latency fetched first since the
       safety snapshot below also needs its last_deadline_violated
       (telemetry_t has no such field - see safety_supervisor.h). ---- */
    safety_supervisor_get_stats(&out->safety_latency);
    sensor_monitor_get_stats(&out->sensor_latency);
    navigation_get_stats(&out->navigation_latency);
    motor_controller_get_stats(&out->motor_latency);
    display_get_stats(&out->display_latency);

    /* ---- safety (item 17) ---- */
    out->safety.state                     = t.state;
    out->safety.requested_speed_percent   = t.requested_speed_percent;
    out->safety.approved_speed_percent    = t.approved_speed_percent;
    out->safety.safety_processing_time_ms = t.processing_time_ms;
    out->safety.safety_deadline_bound_ms  = t.deadline_bound_ms;
    out->safety.safety_deadline_violated  = out->safety_latency.last_deadline_violated;
    out->safety.snapshot_stale            = t.snapshot_stale;
    out->safety.manual_estop_active       = t.manual_estop_active;
    build_decision_reason(out->safety.decision_reason,
                           sizeof(out->safety.decision_reason), &t,
                           out->safety_latency.last_deadline_violated);

    /* ---- ultrasonic (item 18) - both channels share one
       recovery-manager entry ("hcsr04"), see the header note. ---- */
    recovery_status_t hcsr04_rs;
    int have_hcsr04_rs = (recovery_manager_get_recovery_status("hcsr04", &hcsr04_rs) == 0);
    for (int i = 0; i < 2; i++)
    {
        out->ultrasonic[i].distance_cm       = t.ultrasonic_cm[i];
        out->ultrasonic[i].health            = t.ultrasonic_health[i];
        out->ultrasonic[i].valid             = (t.ultrasonic_health[i] != SENSOR_FAULT);
        out->ultrasonic[i].stale             = t.snapshot_stale;
        out->ultrasonic[i].last_update_age_ms = now_ms() - t.last_update_ms;
        out->ultrasonic[i].recovery_state    = have_hcsr04_rs ? hcsr04_rs.state : RECOVERY_STATE_NONE;
        out->ultrasonic[i].fault_count       = have_hcsr04_rs ? hcsr04_rs.failure_count : 0;
    }

    /* ---- IR (item 19) ---- */
    out->ir.lane_position       = t.lane_position;
    out->ir.lane_health         = t.lane_health;
    out->ir.last_update_age_ms  = now_ms() - t.last_update_ms;

    /* ---- encoder (item 20) ---- */
    out->encoder.left_rpm          = t.left_rpm;
    out->encoder.right_rpm         = t.right_rpm;
    out->encoder.health            = t.encoder_health;
    out->encoder.left_direction    = dir_from_rpm(t.left_rpm);
    out->encoder.right_direction   = dir_from_rpm(t.right_rpm);
    out->encoder.last_update_age_ms = now_ms() - t.last_update_ms;

    /* ---- IMU (item 21) ---- */
    out->imu.accel_x = t.accel_x;
    out->imu.accel_y = t.accel_y;
    out->imu.accel_z = t.accel_z;
    out->imu.health  = t.imu_health;
    out->imu.last_update_age_ms = now_ms() - t.last_update_ms;

    /* ---- motor (item 22) ---- */
    motor_status_t ms;
    motor_controller_get_status(&ms);
    out->motor.requested_motor_speed = ms.requested_motor_speed;
    out->motor.approved_motor_speed  = ms.approved_motor_speed;
    out->motor.left_direction        = ms.left_direction;
    out->motor.right_direction       = ms.right_direction;
    out->motor.emergency_stop        = ms.emergency_stop;
    out->motor.command_sequence      = ms.sequence;
    out->motor.command_timestamp_ms  = ms.timestamp_ms;
    watchdog_status_t motor_wd;
    out->motor.controller_health = (recovery_manager_get_watchdog_status(MODULE_MOTOR_CONTROLLER, &motor_wd) == 0)
                                  ? motor_wd.state : WATCHDOG_HEALTHY;

    /* ---- navigation (item 23) ---- */
    navigation_status_t ns;
    navigation_get_status(&ns);
    out->navigation.command                 = ns.command;
    out->navigation.requested_speed_percent = ns.requested_speed_percent;
    out->navigation.health                  = ns.health;

    /* ---- watchdog (item 24) ---- */
    fill_watchdog(&out->watchdog[MODULE_NAVIGATION],       MODULE_NAVIGATION);
    fill_watchdog(&out->watchdog[MODULE_SAFETY],           MODULE_SAFETY);
    fill_watchdog(&out->watchdog[MODULE_SENSOR_MONITOR],   MODULE_SENSOR_MONITOR);
    fill_watchdog(&out->watchdog[MODULE_MOTOR_CONTROLLER], MODULE_MOTOR_CONTROLLER);

    /* ---- recovery (item 25) ---- */
    static const char *recovery_names[TELEMETRY_RECOVERY_ENTRIES] =
        { "hcsr04", "ir_array", "encoder", "imu" };
    for (int i = 0; i < TELEMETRY_RECOVERY_ENTRIES; i++)
    {
        fill_recovery(&out->recovery[i], recovery_names[i]);
    }
}

/* One upload attempt for this cycle: serializes the just-built
   snapshot, POSTs it, then drains and POSTs a batch of queued events
   (item 38 - "upload queued events, then latest snapshot" on
   reconnect is naturally satisfied here too, since a queued event
   batch is attempted every cycle regardless of whether THIS cycle is
   the first one back online). Updates the connection state machine
   and performance stats. Never called when should_attempt_upload()
   says not to (offline/backing off) - see the call site below. */
static void run_upload_cycle(const telemetry_snapshot_t *snap)
{
    static char json_buf[TELEMETRY_MAX_PAYLOAD_BYTES];

    uint64_t serialize_start = now_ms();
    int json_len = telemetry_protocol_build_snapshot_json(snap, json_buf, sizeof(json_buf));
    float serialization_ms = (float)(now_ms() - serialize_start);

    if (json_len < 0)
    {
        /* Snapshot didn't fit TELEMETRY_MAX_PAYLOAD_BYTES - treat as
           a failed upload rather than sending a truncated/corrupt
           JSON body. Should not happen in practice (see config.h's
           sizing comment) but fail safe rather than silently
           truncating data a dashboard would then misparse. */
        report_upload_result(0);
        return;
    }

    telemetry_transport_result_t snap_result;
    uint64_t upload_start = now_ms();
    telemetry_transport_post("robot_telemetry", json_buf, (size_t)json_len, &snap_result);

    int overall_success = snap_result.success;

    /* Only bother with the event batch if the snapshot upload itself
       worked - no point spending a second round-trip on a link that
       just failed; the events stay queued for next cycle. */
    if (snap_result.success)
    {
        telemetry_event_record_t events[TELEMETRY_EVENT_BATCH_MAX];
        int n = telemetry_drain_events(events, TELEMETRY_EVENT_BATCH_MAX);
        if (n > 0)
        {
            static char event_json_buf[TELEMETRY_MAX_PAYLOAD_BYTES];
            int elen = telemetry_protocol_build_events_json(events, n, event_json_buf,
                                                              sizeof(event_json_buf));
            if (elen >= 0)
            {
                telemetry_transport_result_t evt_result;
                telemetry_transport_post("robot_events", event_json_buf, (size_t)elen, &evt_result);
                overall_success = evt_result.success;
                /* Events already popped from the queue above even if
                   this upload fails - see the file header comment on
                   telemetry_drain_events() in telemetry_service.h:
                   this queue is a best-effort CLOUD copy, not the
                   authoritative local history (that's telemetry.h's
                   event ring, untouched by any of this). */
            }
        }
    }

    uint64_t upload_done_ms = now_ms();

    pthread_mutex_lock(&stats_lock);
    published_stats.serialization_ms   = serialization_ms;
    published_stats.network_connect_ms = (float)snap_result.connect_ms;
    published_stats.network_send_ms    = (float)snap_result.send_ms;
    published_stats.server_response_ms = (float)snap_result.response_ms;
    published_stats.total_upload_ms    = (float)(upload_done_ms - upload_start);
    pthread_mutex_unlock(&stats_lock);

    report_upload_result(overall_success);

    if (!overall_success)
    {
        /* Don't leave a possibly-half-broken persistent connection
           around across the backoff wait - next attempt starts
           clean (item 36). */
        telemetry_transport_close();
    }
}

/* ================================================================
 * Service thread - lowest SCHED_FIFO priority in the system.
 * ================================================================ */

static void *telemetry_service_thread(void *arg)
{
    (void)arg;

    telemetry_transport_init(); /* logs once if unconfigured; never fatal */

    uint64_t boot_time_ms = now_ms();
    uint32_t boot_id       = (uint32_t)(boot_time_ms ^ (uint64_t)getpid());
    uint64_t sequence      = 0;

    uint64_t last_cycle_start_ms = boot_time_ms;

    for (;;)
    {
        struct timespec ts;
        ts.tv_sec  = TELEMETRY_SNAPSHOT_INTERVAL_MS / 1000;
        ts.tv_nsec = (long)(TELEMETRY_SNAPSHOT_INTERVAL_MS % 1000) * 1000000L;
        nanosleep(&ts, NULL);

        uint64_t cycle_start_ms = now_ms();
        float jitter_ms = (float)((int64_t)(cycle_start_ms - last_cycle_start_ms)
                                   - TELEMETRY_SNAPSHOT_INTERVAL_MS);
        if (jitter_ms < 0) jitter_ms = -jitter_ms;
        last_cycle_start_ms = cycle_start_ms;

        sequence++;

        telemetry_snapshot_t snap;
        assemble_snapshot(&snap, boot_time_ms, boot_id, sequence);

        uint64_t build_done_ms = now_ms();
        float build_ms = (float)(build_done_ms - cycle_start_ms);

        pthread_mutex_lock(&snapshot_lock);
        published_snapshot = snap;
        pthread_mutex_unlock(&snapshot_lock);

        pthread_mutex_lock(&stats_lock);
        published_stats.snapshot_build_ms          = build_ms;
        published_stats.telemetry_thread_jitter_ms = jitter_ms;
        pthread_mutex_unlock(&stats_lock);

        if (should_attempt_upload())
        {
            run_upload_cycle(&snap);
        }
        /* else: OFFLINE (unconfigured) or still within a BACKOFF
           window - local operation is completely unaffected either
           way (item 37); the event queue just keeps accumulating up
           to TELEMETRY_EVENT_QUEUE_CAPACITY, dropping DEBUG-first
           past that (item 29). */
    }

    return NULL; /* unreachable */
}

void telemetry_service_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_TELEMETRY;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, telemetry_service_thread, NULL) != 0)
    {
        logger_log_err("telemetry_service_start: pthread_create failed - "
                       "telemetry snapshots will not be produced");
        return;
    }
    pthread_detach(tid);
}
