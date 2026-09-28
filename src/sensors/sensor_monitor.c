#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/neutrino.h>

#include "sensor_monitor.h"
#include "sensor_health.h"
#include "line_position.h"
#include "../common/messages.h"
#include "../common/config.h"
#include "../common/watchdog.h"
#include "../common/logger.h"
#include "../drivers/hcsr04_driver.h"
#include "../drivers/encoder_driver.h"
#include "../drivers/imu_driver.h"
#include "../safety/safety_supervisor.h"
#include "../system/recovery_manager.h"

/* Recovery flags: set by Recovery Manager (from ITS thread) once a
   recovery attempt for that sensor has succeeded. Sensor Monitor
   polls these each tick and, if set, resets its OWN local filter
   state for that sensor (Recovery Manager only re-inits the driver
   itself - it doesn't know about sensor_ctx_t's history). */
static atomic_int hcsr04_recovered   = 0;
static atomic_int ir_recovered       = 0;
static atomic_int encoder_recovered  = 0;
static atomic_int imu_recovered      = 0;

/* Published stats snapshot - see sensor_monitor_get_stats() in the
   header. Updated once per completed STATS_WINDOW_TICKS window, the
   same moment the existing "[SensorMonitor][stats]" log line fires.
   Never nested with any other lock; held only for the struct copy. */
static pthread_mutex_t stats_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static sensor_monitor_stats_t published_stats;

static int recover_hcsr04(void)
{
    hcsr04_driver_deinit();
    return hcsr04_driver_init();
}

static int recover_ir(void)
{
    line_position_deinit();
    return line_position_init();
}

static int recover_encoder(void)
{
    encoder_driver_deinit();
    return encoder_driver_init();
}

static int recover_imu(void)
{
    imu_driver_deinit();
    return imu_driver_init();
}

static float read_ultrasonic(int index, sensor_ctx_t *ctx)
{
    float samples[SAMPLES_PER_TICK];
    for (int i = 0; i < SAMPLES_PER_TICK; i++)
        samples[i] = hcsr04_read_raw_cm(index);
    return sensor_ctx_process(ctx, samples, SAMPLES_PER_TICK);
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static float elapsed_ms(struct timespec *start, struct timespec *end)
{
    float sec_part  = (float)(end->tv_sec - start->tv_sec) * 1000.0f;
    float nsec_part = (float)(end->tv_nsec - start->tv_nsec) / 1000000.0f;
    return sec_part + nsec_part;
}

/* Simple running min/max/avg over a fixed window - see
   STATS_WINDOW_TICKS in config.h. Printed periodically rather than
   spamming a line every cycle (item 6/13). */
typedef struct { float sum, min, max; int count; } stats_t;
static void stats_reset(stats_t *s) { s->sum = 0; s->min = 1.0e9f; s->max = 0; s->count = 0; }
static void stats_add(stats_t *s, float v)
{
    s->sum += v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->count++;
}

static void *sensor_monitor_thread(void *arg)
{
    (void)arg;

    if (hcsr04_driver_init() != 0)
        recovery_manager_report_fault("hcsr04", &hcsr04_recovered);

    if (line_position_init() != 0)
        recovery_manager_report_fault("ir_array", &ir_recovered);

    int encoder_ok = (encoder_driver_init() == 0);
    if (!encoder_ok)
        recovery_manager_report_fault("encoder", &encoder_recovered);

    int imu_ok = (imu_driver_init() == 0);
    if (!imu_ok)
        recovery_manager_report_fault("imu", &imu_recovered);

    sensor_ctx_t us0, us1;
    sensor_ctx_init(&us0);
    sensor_ctx_init(&us1);

    int safety_chid = safety_supervisor_get_chid();
    int safety_coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_chid,
                                      _NTO_SIDE_CHANNEL, 0);

    int rm_coid = ConnectAttach(ND_LOCAL_NODE, 0, recovery_manager_get_chid(),
                                  _NTO_SIDE_CHANNEL, 0);

    stats_t cycle_stats, us_stats, ir_stats, encoder_stats, imu_stats;
    stats_reset(&cycle_stats);
    stats_reset(&us_stats);
    stats_reset(&ir_stats);
    stats_reset(&encoder_stats);
    stats_reset(&imu_stats);

    for (;;)
    {
        struct timespec cycle_start, cycle_end;
        struct timespec phase_a, phase_b;
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);

        /* Recovery Manager only decides WHEN a retry is due; SensorMonitor
           is the sole owner of these drivers and is the ONLY thread ever
           allowed to call their deinit()/init(). Poll on our own thread,
           run recover() in-line right here if due, and report the
           outcome back so Recovery Manager can track attempts/backoff. */
        if (recovery_manager_poll_pending("hcsr04"))
        {
            int rc = recover_hcsr04();
            recovery_manager_submit_result("hcsr04",
                rc == 0 ? RECOVERY_RESULT_SUCCESS : RECOVERY_RESULT_FAILURE);
        }
        if (recovery_manager_poll_pending("ir_array"))
        {
            int rc = recover_ir();
            recovery_manager_submit_result("ir_array",
                rc == 0 ? RECOVERY_RESULT_SUCCESS : RECOVERY_RESULT_FAILURE);
        }
        if (recovery_manager_poll_pending("encoder"))
        {
            int rc = recover_encoder();
            recovery_manager_submit_result("encoder",
                rc == 0 ? RECOVERY_RESULT_SUCCESS : RECOVERY_RESULT_FAILURE);
        }
        if (recovery_manager_poll_pending("imu"))
        {
            int rc = recover_imu();
            recovery_manager_submit_result("imu",
                rc == 0 ? RECOVERY_RESULT_SUCCESS : RECOVERY_RESULT_FAILURE);
        }

        /* Pick up any completed recoveries (including ones just run
           above) and reset our own local filter state for that sensor. */
        if (atomic_exchange(&hcsr04_recovered, 0))
        {
            sensor_ctx_init(&us0);
            sensor_ctx_init(&us1);
        }
        if (atomic_exchange(&ir_recovered, 0)) { /* line_position has no
                                                     local state to reset */ }
        if (atomic_exchange(&encoder_recovered, 0)) encoder_ok = 1;
        if (atomic_exchange(&imu_recovered, 0))     imu_ok = 1;

        safety_message_t msg;
        msg.type = MSG_SENSOR_DATA;

        clock_gettime(CLOCK_MONOTONIC, &phase_a);
        msg.sensor.ultrasonic_cm[0] = read_ultrasonic(0, &us0);
        msg.sensor.ultrasonic_cm[1] = read_ultrasonic(1, &us1);
        msg.sensor.ultrasonic_health[0] = us0.health;
        msg.sensor.ultrasonic_health[1] = us1.health;
        clock_gettime(CLOCK_MONOTONIC, &phase_b);
        stats_add(&us_stats, elapsed_ms(&phase_a, &phase_b));

        clock_gettime(CLOCK_MONOTONIC, &phase_a);
        line_position_read(&msg.sensor.lane_position, &msg.sensor.lane_health);
        clock_gettime(CLOCK_MONOTONIC, &phase_b);
        stats_add(&ir_stats, elapsed_ms(&phase_a, &phase_b));

        float left_rpm = 0.0f, right_rpm = 0.0f;
        clock_gettime(CLOCK_MONOTONIC, &phase_a);
        if (encoder_ok)
        {
            encoder_driver_read_rpm(&left_rpm, &right_rpm);
        }
        clock_gettime(CLOCK_MONOTONIC, &phase_b);
        stats_add(&encoder_stats, elapsed_ms(&phase_a, &phase_b));
        msg.sensor.left_rpm       = left_rpm;
        msg.sensor.right_rpm      = right_rpm;
        msg.sensor.encoder_health = encoder_ok ? SENSOR_OK : SENSOR_FAULT;

        float ax = 0, ay = 0, az = 1.0f;
        clock_gettime(CLOCK_MONOTONIC, &phase_a);
        if (imu_ok && imu_driver_read_accel(&ax, &ay, &az) != 0)
        {
            imu_ok = 0;
            recovery_manager_report_fault("imu", &imu_recovered);
        }
        clock_gettime(CLOCK_MONOTONIC, &phase_b);
        stats_add(&imu_stats, elapsed_ms(&phase_a, &phase_b));
        msg.sensor.accel_x = ax;
        msg.sensor.accel_y = ay;
        msg.sensor.accel_z = az;
        msg.sensor.imu_health = imu_ok ? SENSOR_OK : SENSOR_FAULT;

        /* Timestamp the finished snapshot - this is what lets Safety
           tell "fresh" apart from "stale" (item 9) instead of trusting
           a once-good reading forever. */
        msg.sensor.snapshot_time_ms = now_ms();

        /* Escalate to Recovery Manager the moment a sensor crosses
           into FAULT, not every tick after that - report_fault()
           already de-duplicates internally, but checking health here
           first avoids spamming the call every single tick. */
        if (us0.health == SENSOR_FAULT || us1.health == SENSOR_FAULT)
            recovery_manager_report_fault("hcsr04", &hcsr04_recovered);
        if (msg.sensor.lane_health == SENSOR_FAULT)
            recovery_manager_report_fault("ir_array", &ir_recovered);

        /* Heartbeat pulsed here - as soon as this cycle's reads are
           done and the snapshot is built - rather than after Safety's
           ack comes back. Safety's MSG_SENSOR_DATA handler only ever
           copies the struct and replies immediately (see
           safety_supervisor.c), so this send is normally sub-
           millisecond; pulsing before it removes even that small,
           avoidable coupling (item 1). */
        MsgSendPulse(rm_coid, -1, PULSE_CODE_HEARTBEAT, MODULE_SENSOR_MONITOR);

        ack_reply_t ack;
        MsgSend(safety_coid, &msg, sizeof(msg), &ack, sizeof(ack));

        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        stats_add(&cycle_stats, elapsed_ms(&cycle_start, &cycle_end));

        if (VERBOSE_TICK_LOGGING)
        {
            logger_log("[SensorMonitor] cycle=%.1fms",
                   elapsed_ms(&cycle_start, &cycle_end));
        }

        if (cycle_stats.count >= STATS_WINDOW_TICKS)
        {
            logger_log("[SensorMonitor][stats] cycle min/max/avg=%.2f/%.2f/%.2fms  "
                   "us min/max/avg=%.2f/%.2f/%.2fms  "
                   "ir min/max/avg=%.2f/%.2f/%.2fms  "
                   "encoder min/max/avg=%.2f/%.2f/%.2fms  "
                   "imu min/max/avg=%.2f/%.2f/%.2fms (window=%d)",
                   cycle_stats.min, cycle_stats.max, cycle_stats.sum / cycle_stats.count,
                   us_stats.min, us_stats.max, us_stats.sum / us_stats.count,
                   ir_stats.min, ir_stats.max, ir_stats.sum / ir_stats.count,
                   encoder_stats.min, encoder_stats.max, encoder_stats.sum / encoder_stats.count,
                   imu_stats.min, imu_stats.max, imu_stats.sum / imu_stats.count,
                   cycle_stats.count);

            pthread_mutex_lock(&stats_pub_lock);
            #define PUBLISH(field, src) \
                published_stats.field.min_ms = (src).min; \
                published_stats.field.max_ms = (src).max; \
                published_stats.field.avg_ms = (src).count ? (src).sum / (src).count : 0.0f; \
                published_stats.field.samples = (src).count;
            PUBLISH(cycle,     cycle_stats)
            PUBLISH(ultrasonic, us_stats)
            PUBLISH(ir,        ir_stats)
            PUBLISH(encoder,   encoder_stats)
            PUBLISH(imu,       imu_stats)
            #undef PUBLISH
            pthread_mutex_unlock(&stats_pub_lock);

            stats_reset(&cycle_stats);
            stats_reset(&us_stats);
            stats_reset(&ir_stats);
            stats_reset(&encoder_stats);
            stats_reset(&imu_stats);
        }
    }

    return NULL; /* unreachable */
}

void sensor_monitor_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_SENSOR_MONITOR;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, sensor_monitor_thread, NULL) != 0)
    {
        perror("sensor_monitor_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);
}

void sensor_monitor_get_stats(sensor_monitor_stats_t *out)
{
    pthread_mutex_lock(&stats_pub_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_pub_lock);
}
