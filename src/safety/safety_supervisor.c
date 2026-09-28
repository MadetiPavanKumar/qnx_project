#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sys/neutrino.h>

#include "safety_supervisor.h"
#include "../common/messages.h"
#include "../common/config.h"
#include "../common/watchdog.h"
#include "../common/labels.h"
#include "../common/logger.h"
#include "../motor/motor_controller.h"
#include "../system/recovery_manager.h"
#include "../system/telemetry.h"
#include "../system/cli.h"

/* NOTE: status_display.h/OLED is intentionally NOT included here
   anymore. Safety must never perform display/I2C operations directly
   - that work now lives entirely in display/display_task.c, running
   on its own low-priority thread that only reads telemetry_get().
   This removes the single biggest source of the false watchdog
   "missed heartbeat" storm: devctl() calls have no bounded timeout
   here, and a stalled OLED write used to stall this thread directly,
   cascading into Navigation's reply and Motor Controller's command
   both arriving late in the same tick. */

/* Simple running min/max/avg over a fixed window - see
   STATS_WINDOW_TICKS in config.h. Printed periodically instead of
   every tick, per the "reduce real-time console printing" fix;
   nothing here does console I/O on the hot path. */
typedef struct
{
    float sum, min, max;
    int   count;
} stats_t;

static void stats_reset(stats_t *s)
{
    s->sum = 0.0f;
    s->min = 1.0e9f;
    s->max = 0.0f;
    s->count = 0;
}

static void stats_add(stats_t *s, float value)
{
    s->sum += value;
    if (value < s->min) s->min = value;
    if (value > s->max) s->max = value;
    s->count++;
}

static int safety_chid = -1;
static pthread_mutex_t init_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  init_cond = PTHREAD_COND_INITIALIZER;
static int channel_ready = 0;

/* Published stats snapshot - see safety_supervisor_get_stats() in the
   header. Updated in two places below: last_deadline_violated every
   tick (it's "most recent tick" info, not a windowed average), and
   cycle/processing once per completed STATS_WINDOW_TICKS window (the
   same moment the existing "[Safety][stats]" log line fires) - so a
   getter always returns either "no window yet" (samples==0) or one
   complete, real window, never a partially-accumulated one. This
   mutex is only ever held for the few floats being copied in/out -
   never nested with state_lock, telemetry_lock, or held across any
   IPC/hardware call. */
static pthread_mutex_t stats_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static safety_stats_t published_stats;

static float elapsed_ms(struct timespec *start, struct timespec *end)
{
    float sec_part  = (float)(end->tv_sec - start->tv_sec) * 1000.0f;
    float nsec_part = (float)(end->tv_nsec - start->tv_nsec) / 1000000.0f;
    return sec_part + nsec_part;
}

/* One subsystem's verdict: how bad is it, and what fraction of the
   requested speed does IT allow (independent of the others). The
   overall decision takes the worst state and the smallest fraction
   across all subsystems - "supervisor overrides unsafe commands"
   means the most restrictive voice wins, always. */
typedef struct { safety_state_t state; float fraction; } subsystem_result_t;

static int severity_rank(safety_state_t s)
{
    switch (s)
    {
        case SAFETY_FAULT:     return 3;
        case SAFETY_EMERGENCY: return 2;
        case SAFETY_WARNING:   return 1;
        case SAFETY_SAFE:
        default:                return 0;
    }
}

static subsystem_result_t check_ultrasonic(const sensor_snapshot_t *snap)
{
    subsystem_result_t r;

    if (snap->ultrasonic_health[0] == SENSOR_FAULT ||
        snap->ultrasonic_health[1] == SENSOR_FAULT)
    {
        r.state = SAFETY_FAULT; r.fraction = 0.0f;
        return r;
    }

    float min_distance = (snap->ultrasonic_cm[0] < snap->ultrasonic_cm[1])
                        ? snap->ultrasonic_cm[0] : snap->ultrasonic_cm[1];

    if (min_distance <= EMERGENCY_DISTANCE_CM)
    {
        r.state = SAFETY_EMERGENCY; r.fraction = 0.0f;
    }
    else if (min_distance <= WARNING_DISTANCE_CM)
    {
        float span     = WARNING_DISTANCE_CM - EMERGENCY_DISTANCE_CM;
        float fraction = (min_distance - EMERGENCY_DISTANCE_CM) / span;
        if (fraction < 0.0f) fraction = 0.0f;
        if (fraction > 1.0f) fraction = 1.0f;
        r.state = SAFETY_WARNING; r.fraction = fraction;
    }
    else
    {
        r.state = SAFETY_SAFE; r.fraction = 1.0f;
    }
    return r;
}

/* TEST VALUES - see config.h. Detects a sudden jolt (accel magnitude
   spike) or excessive tilt (low az), either of which suggests a
   collision or a tip-over in progress. */
static subsystem_result_t check_imu(const sensor_snapshot_t *snap)
{
    subsystem_result_t r;

    if (snap->imu_health == SENSOR_FAULT)
    {
        r.state = SAFETY_FAULT; r.fraction = 0.0f;
        return r;
    }

    float mag = sqrtf(snap->accel_x * snap->accel_x +
                       snap->accel_y * snap->accel_y +
                       snap->accel_z * snap->accel_z);

    if (mag > IMU_MAX_ACCEL_MAGNITUDE_G || snap->accel_z < IMU_MIN_UPRIGHT_AZ_G)
    {
        r.state = SAFETY_EMERGENCY; r.fraction = 0.0f;
        return r;
    }

    r.state = SAFETY_SAFE; r.fraction = 1.0f;
    return r;
}

/* TEST VALUES - see config.h. Overspeed, or one wheel spinning much
   faster than the other (possible stall/slip on the slower one). */
static subsystem_result_t check_encoder(const sensor_snapshot_t *snap)
{
    subsystem_result_t r;

    if (snap->encoder_health == SENSOR_FAULT)
    {
        r.state = SAFETY_FAULT; r.fraction = 0.0f;
        return r;
    }

    float left  = snap->left_rpm  < 0 ? -snap->left_rpm  : snap->left_rpm;
    float right = snap->right_rpm < 0 ? -snap->right_rpm : snap->right_rpm;
    float faster = (left > right) ? left : right;
    float slower = (left < right) ? left : right;

    if (faster > ENCODER_MAX_RPM)
    {
        r.state = SAFETY_EMERGENCY; r.fraction = 0.0f;
        return r;
    }

    if (slower > 1.0f && (faster / slower) > ENCODER_IMBALANCE_RATIO)
    {
        r.state = SAFETY_WARNING; r.fraction = 0.5f;
        return r;
    }

    if (faster > ENCODER_WARNING_RPM)
    {
        float fraction = 1.0f - ((faster - ENCODER_WARNING_RPM) /
                                  (ENCODER_MAX_RPM - ENCODER_WARNING_RPM));
        if (fraction < 0.0f) fraction = 0.0f;
        if (fraction > 1.0f) fraction = 1.0f;
        r.state = SAFETY_WARNING; r.fraction = fraction;
        return r;
    }

    r.state = SAFETY_SAFE; r.fraction = 1.0f;
    return r;
}

static void evaluate_safety(const sensor_snapshot_t *snap, int requested_speed,
                             safety_state_t *out_state, int *out_approved_speed)
{
    subsystem_result_t u = check_ultrasonic(snap);
    subsystem_result_t i = check_imu(snap);
    subsystem_result_t e = check_encoder(snap);

    subsystem_result_t *worst = &u;
    if (severity_rank(i.state) > severity_rank(worst->state)) worst = &i;
    if (severity_rank(e.state) > severity_rank(worst->state)) worst = &e;

    float fraction = u.fraction;
    if (i.fraction < fraction) fraction = i.fraction;
    if (e.fraction < fraction) fraction = e.fraction;

    if (cli_is_estop_active())
    {
        *out_state = SAFETY_EMERGENCY;
        *out_approved_speed = 0;
        return;
    }

    *out_state = worst->state;
    *out_approved_speed = (int)(requested_speed * fraction);
}

static void build_motor_command(navigation_command_t cmd, int approved_speed,
                                 motor_command_t *out)
{
    if (approved_speed <= 0)
    {
        out->left_dir = WHEEL_STOP; out->right_dir = WHEEL_STOP;
        out->speed_percent = 0;
        return;
    }

    switch (cmd)
    {
        case CMD_FORWARD:
            out->left_dir = WHEEL_FORWARD; out->right_dir = WHEEL_FORWARD;
            break;
        case CMD_BACKWARD:
            out->left_dir = WHEEL_REVERSE; out->right_dir = WHEEL_REVERSE;
            break;
        case CMD_LEFT:
            out->left_dir = WHEEL_REVERSE; out->right_dir = WHEEL_FORWARD;
            break;
        case CMD_RIGHT:
            out->left_dir = WHEEL_FORWARD; out->right_dir = WHEEL_REVERSE;
            break;
        case CMD_STOP:
        default:
            out->left_dir = WHEEL_STOP; out->right_dir = WHEEL_STOP;
            approved_speed = 0;
            break;
    }
    out->speed_percent = (unsigned int)approved_speed;
}

static void *safety_supervisor_thread(void *arg)
{
    (void)arg;

    int chid = ChannelCreate(0);
    if (chid == -1)
    {
        perror("safety_supervisor: ChannelCreate failed");
        exit(EXIT_FAILURE);
    }

    pthread_mutex_lock(&init_lock);
    safety_chid = chid;
    channel_ready = 1;
    pthread_cond_signal(&init_cond);
    pthread_mutex_unlock(&init_lock);

    int motor_coid = ConnectAttach(ND_LOCAL_NODE, 0, motor_controller_get_chid(),
                                     _NTO_SIDE_CHANNEL, 0);
    int rm_coid = ConnectAttach(ND_LOCAL_NODE, 0, recovery_manager_get_chid(),
                                  _NTO_SIDE_CHANNEL, 0);

    sensor_snapshot_t latest_snapshot;
    latest_snapshot.ultrasonic_cm[0] = 0.0f;
    latest_snapshot.ultrasonic_cm[1] = 0.0f;
    latest_snapshot.ultrasonic_health[0] = SENSOR_FAULT;
    latest_snapshot.ultrasonic_health[1] = SENSOR_FAULT;
    latest_snapshot.lane_position = LANE_UNKNOWN;
    latest_snapshot.lane_health   = SENSOR_FAULT;
    latest_snapshot.left_rpm = 0; latest_snapshot.right_rpm = 0;
    latest_snapshot.encoder_health = SENSOR_FAULT;
    latest_snapshot.accel_x = 0; latest_snapshot.accel_y = 0; latest_snapshot.accel_z = 1.0f;
    latest_snapshot.imu_health = SENSOR_FAULT;
    latest_snapshot.snapshot_time_ms = 0; /* age-checked below; 0 reads as stale until real data arrives */

    safety_state_t prev_logged_state = SAFETY_SAFE;

    stats_t period_stats, exec_stats;
    stats_reset(&period_stats);
    stats_reset(&exec_stats);
    float worst_latency_ms = 0.0f;
    struct timespec prev_tick_start;
    int have_prev_tick_start = 0;

    safety_message_t msg;

    for (;;)
    {
        int rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) continue;
        if (rcvid == 0)  continue;

        if (msg.type == MSG_SENSOR_DATA)
        {
            latest_snapshot = msg.sensor;
            ack_reply_t ack = { .ok = 1 };
            MsgReply(rcvid, EOK, &ack, sizeof(ack));
            continue;
        }

        if (msg.type != MSG_NAVIGATION)
        {
            MsgError(rcvid, ENOSYS);
            continue;
        }

        struct timespec tick_start, tick_end;
        clock_gettime(CLOCK_MONOTONIC, &tick_start);

        if (have_prev_tick_start)
            stats_add(&period_stats, elapsed_ms(&prev_tick_start, &tick_start));
        prev_tick_start = tick_start;
        have_prev_tick_start = 1;

        safety_state_t state;
        int approved_speed;
        evaluate_safety(&latest_snapshot, msg.navigation.requested_speed_percent,
                         &state, &approved_speed);

        /* Stale-data fail-safe (item 9): a snapshot that was once
           valid must not be trusted forever. If Sensor Monitor's last
           delivered snapshot is older than SENSOR_SNAPSHOT_STALE_MS,
           treat it the same as a sensor FAULT regardless of what its
           last numbers said. */
        uint64_t now_ms_val = (uint64_t)tick_start.tv_sec * 1000ULL
                             + (uint64_t)(tick_start.tv_nsec / 1000000ULL);
        uint64_t snapshot_age_ms = (latest_snapshot.snapshot_time_ms == 0)
                                 ? UINT64_MAX
                                 : now_ms_val - latest_snapshot.snapshot_time_ms;
        int snapshot_stale = (snapshot_age_ms > SENSOR_SNAPSHOT_STALE_MS);
        if (snapshot_stale)
        {
            state = SAFETY_FAULT;
            approved_speed = 0;
        }

        clock_gettime(CLOCK_MONOTONIC, &tick_end);
        float processing_ms = elapsed_ms(&tick_start, &tick_end);

        /* Real, enforced safety deadline (item 15) - not just a
           reported number. If the decision itself ran long, don't
           approve a command computed from a stale decision: fail
           safe on the spot and record why. */
        int deadline_violated = (processing_ms > (float)SAFETY_DEADLINE_MS);
        if (deadline_violated)
        {
            state = SAFETY_FAULT;
            approved_speed = 0;
        }

        pthread_mutex_lock(&stats_pub_lock);
        published_stats.deadline_ms             = SAFETY_DEADLINE_MS;
        published_stats.last_deadline_violated   = deadline_violated;
        pthread_mutex_unlock(&stats_pub_lock);

        stats_add(&exec_stats, processing_ms);
        if (processing_ms > worst_latency_ms) worst_latency_ms = processing_ms;

        safety_reply_t reply;
        reply.state                  = state;
        reply.approved_speed_percent = approved_speed;
        reply.processing_time_ms     = processing_ms;
        reply.deadline_bound_ms      = TICK_INTERVAL_MS;
        reply.ultrasonic_health[0]   = latest_snapshot.ultrasonic_health[0];
        reply.ultrasonic_health[1]   = latest_snapshot.ultrasonic_health[1];
        reply.lane_position          = latest_snapshot.lane_position;
        reply.lane_health            = latest_snapshot.lane_health;

        MsgReply(rcvid, EOK, &reply, sizeof(reply));

        /* Heartbeat is sent HERE - right after Safety has completed
           its own decision and replied to Navigation - rather than
           after the Motor Controller round trip below. This is the
           fix for heartbeat/IPC coupling (item 1): Safety's pulse now
           represents "I evaluated this tick and replied," not "the
           downstream Motor Controller round trip also finished." */
        MsgSendPulse(rm_coid, -1, PULSE_CODE_HEARTBEAT, MODULE_SAFETY);

        motor_command_t motor_cmd;
        build_motor_command(msg.navigation.command, approved_speed, &motor_cmd);
        motor_reply_t motor_reply;
        MsgSend(motor_coid, &motor_cmd, sizeof(motor_cmd),
                &motor_reply, sizeof(motor_reply));

        /* Log on transition only - avoids spamming an event every
           75ms while sitting in the same state. Covers both "Safety
           Events" and "Override Latency" (measured, not estimated). */
        if (state != prev_logged_state)
        {
            if (state != SAFETY_SAFE)
            {
                event_log_add("State -> %s (requested %d%%, approved %d%%, "
                               "latency %.2fms)%s%s%s",
                               safety_state_label(state),
                               msg.navigation.requested_speed_percent,
                               approved_speed, processing_ms,
                               cli_is_estop_active() ? " [manual estop]" : "",
                               snapshot_stale ? " [stale sensor data]" : "",
                               deadline_violated ? " [DEADLINE VIOLATION]" : "");
            }
            else
            {
                event_log_add("State -> SAFE (cleared)");
            }
            prev_logged_state = state;
        }
        else if (deadline_violated)
        {
            /* Even if the state was already non-SAFE (so the normal
               transition log above didn't fire), a deadline violation
               is always worth its own event - it's a distinct kind of
               fault from "an obstacle is close." */
            event_log_add("SAFETY DEADLINE VIOLATION (processing %.2fms > "
                           "%dms bound)", processing_ms, SAFETY_DEADLINE_MS);
        }

        telemetry_t t;
        t.last_update_ms          = 0; /* filled by telemetry_update if needed */
        t.state                   = state;
        t.approved_speed_percent  = approved_speed;
        t.requested_speed_percent = msg.navigation.requested_speed_percent;
        t.processing_time_ms      = processing_ms;
        t.deadline_bound_ms       = TICK_INTERVAL_MS;
        t.snapshot_stale          = snapshot_stale;
        t.ultrasonic_cm[0]        = latest_snapshot.ultrasonic_cm[0];
        t.ultrasonic_cm[1]        = latest_snapshot.ultrasonic_cm[1];
        t.ultrasonic_health[0]    = latest_snapshot.ultrasonic_health[0];
        t.ultrasonic_health[1]    = latest_snapshot.ultrasonic_health[1];
        t.lane_position            = latest_snapshot.lane_position;
        t.lane_health              = latest_snapshot.lane_health;
        t.left_rpm                 = latest_snapshot.left_rpm;
        t.right_rpm                = latest_snapshot.right_rpm;
        t.encoder_health            = latest_snapshot.encoder_health;
        t.accel_x = latest_snapshot.accel_x;
        t.accel_y = latest_snapshot.accel_y;
        t.accel_z = latest_snapshot.accel_z;
        t.imu_health                = latest_snapshot.imu_health;
        t.manual_estop_active       = cli_is_estop_active();

        /* Fast, mutex-protected, non-blocking - this is the ONLY
           thing Safety does to publish state outward. The Display
           task (display_task.c) polls telemetry_get() on its own
           low-priority thread and does the (potentially slow) OLED
           I2C writes there - Safety never waits on it. */
        telemetry_update(&t);

        if (VERBOSE_TICK_LOGGING)
        {
            logger_log("[Safety] dist=%.1f/%.1fcm state=%s spd=%d%% "
                   "lat=%.2fms lane=%s rpm=%.0f/%.0f imu_z=%.2f%s",
                   latest_snapshot.ultrasonic_cm[0], latest_snapshot.ultrasonic_cm[1],
                   safety_state_label(state), approved_speed, processing_ms,
                   lane_position_label(latest_snapshot.lane_position),
                   latest_snapshot.left_rpm, latest_snapshot.right_rpm,
                   latest_snapshot.accel_z,
                   t.manual_estop_active ? " [ESTOP]" : "");
        }

        if (exec_stats.count >= STATS_WINDOW_TICKS)
        {
            logger_log("[Safety][stats] period min/max/avg=%.2f/%.2f/%.2fms "
                   "exec min/max/avg=%.2f/%.2f/%.2fms worst=%.2fms "
                   "(window=%d ticks)",
                   period_stats.min, period_stats.max,
                   period_stats.count ? period_stats.sum / period_stats.count : 0.0f,
                   exec_stats.min, exec_stats.max, exec_stats.sum / exec_stats.count,
                   worst_latency_ms, exec_stats.count);

            pthread_mutex_lock(&stats_pub_lock);
            published_stats.cycle.min_ms      = period_stats.min;
            published_stats.cycle.max_ms      = period_stats.max;
            published_stats.cycle.avg_ms      = period_stats.count
                                               ? period_stats.sum / period_stats.count : 0.0f;
            published_stats.cycle.samples     = period_stats.count;
            published_stats.processing.min_ms = exec_stats.min;
            published_stats.processing.max_ms = exec_stats.max;
            published_stats.processing.avg_ms = exec_stats.sum / exec_stats.count;
            published_stats.processing.samples = exec_stats.count;
            pthread_mutex_unlock(&stats_pub_lock);

            stats_reset(&period_stats);
            stats_reset(&exec_stats);
            worst_latency_ms = 0.0f;
        }
    }

    return NULL; /* unreachable */
}

void safety_supervisor_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_SAFETY_SUPERVISOR;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, safety_supervisor_thread, NULL) != 0)
    {
        perror("safety_supervisor_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);

    pthread_mutex_lock(&init_lock);
    while (!channel_ready)
        pthread_cond_wait(&init_cond, &init_lock);
    pthread_mutex_unlock(&init_lock);
}

int safety_supervisor_get_chid(void)
{
    return safety_chid;
}

void safety_supervisor_get_stats(safety_stats_t *out)
{
    pthread_mutex_lock(&stats_pub_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_pub_lock);
}
