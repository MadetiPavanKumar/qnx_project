#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <sys/neutrino.h>

#include <time.h>

#include "motor_controller.h"
#include "../common/messages.h"
#include "../common/config.h"
#include "../common/gpio_map.h"
#include "../common/watchdog.h"
#include "../common/logger.h"
#include "../drivers/drv8833.h"
#include "../system/recovery_manager.h"

static int chid = -1;
static pthread_mutex_t init_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  init_cond = PTHREAD_COND_INITIALIZER;
static int channel_ready = 0;

/* Published snapshots - see motor_controller_get_stats()/
   motor_controller_get_status() in the header. Two independent, tiny
   mutexes: status changes every command (cheap, high frequency),
   stats only once per window. Neither is ever nested with the other,
   with recovery_manager's state_lock, or held across drv8833_*() /
   IPC. */
static pthread_mutex_t stats_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static motor_controller_stats_t published_stats;

static pthread_mutex_t status_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static motor_status_t published_status;
static uint64_t command_sequence = 0;

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

typedef struct { float sum, min, max; int count; } stats_t;
static void stats_reset(stats_t *s) { s->sum = 0; s->min = 1.0e9f; s->max = 0; s->count = 0; }
static void stats_add(stats_t *s, float v)
{
    s->sum += v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->count++;
}

static drv8833_dir_t translate_dir(wheel_dir_t d)
{
    switch (d)
    {
        case WHEEL_FORWARD: return DRV8833_DIR_FORWARD;
        case WHEEL_REVERSE: return DRV8833_DIR_REVERSE;
        case WHEEL_STOP:
        default:             return DRV8833_DIR_STOP;
    }
}

static void *motor_controller_thread(void *arg)
{
    (void)arg;

    chid = ChannelCreate(0);
    if (chid == -1)
    {
        perror("motor_controller: ChannelCreate failed");
        exit(EXIT_FAILURE);
    }

    pthread_mutex_lock(&init_lock);
    channel_ready = 1;
    pthread_cond_signal(&init_cond);
    pthread_mutex_unlock(&init_lock);

    drv8833_config_t cfg;
    cfg.en_pin         = GPIO_DRV8833_EN;
    cfg.right_fwd_pin  = GPIO_DRV8833_RIGHT_FWD;
    cfg.right_rev_pin  = GPIO_DRV8833_RIGHT_REV;
    cfg.left_fwd_pin   = GPIO_DRV8833_LEFT_FWD;
    cfg.left_rev_pin   = GPIO_DRV8833_LEFT_REV;
    cfg.pwm_frequency  = 1000;

    if (drv8833_init(&cfg) != DRV8833_OK)
    {
        fprintf(stderr, "motor_controller: drv8833_init failed - motors "
                        "will not move, but the process stays up so the "
                        "rest of the system (sensing/decisions) still runs\n");
    }

    int rm_coid = ConnectAttach(ND_LOCAL_NODE, 0, recovery_manager_get_chid(),
                                  _NTO_SIDE_CHANNEL, 0);

    motor_command_t cmd;
    motor_reply_t   reply;

    stats_t period_stats, exec_stats;
    stats_reset(&period_stats);
    stats_reset(&exec_stats);
    struct timespec prev_recv;
    int have_prev_recv = 0;

    for (;;)
    {
        int rcvid = MsgReceive(chid, &cmd, sizeof(cmd), NULL);
        if (rcvid == -1) continue;
        if (rcvid == 0)  continue;

        struct timespec recv_time, done_time;
        clock_gettime(CLOCK_MONOTONIC, &recv_time);

        if (have_prev_recv)
            stats_add(&period_stats, elapsed_ms(&prev_recv, &recv_time));
        prev_recv = recv_time;
        have_prev_recv = 1;

        /* Heartbeat sent as soon as we wake up with a command - before
           touching hardware - so it reflects "this thread is alive
           and received work," not "the drv8833 call also finished."
           (item 1: don't gate a heartbeat on more work than proves
           liveness.) drv8833_set_motion() is a fast, mutex-protected,
           non-blocking GPIO/PWM call (audited: no sleeps, no I2C/SPI,
           no devctl()), so in practice this reorder changes little
           here - but it keeps every task in the chain following the
           same rule rather than relying on one driver staying fast
           forever. */
        MsgSendPulse(rm_coid, -1, PULSE_CODE_HEARTBEAT, MODULE_MOTOR_CONTROLLER);

        /* Execute only - Safety already made every safety decision.
           Any hardware failure here fails safe via drv8833's own
           fail-safe apply_motion_locked() (forces a stop on error). */
        drv8833_set_motion(translate_dir(cmd.left_dir),
                            translate_dir(cmd.right_dir),
                            cmd.speed_percent);

        pthread_mutex_lock(&status_pub_lock);
        published_status.requested_motor_speed = cmd.speed_percent;
        published_status.approved_motor_speed  = cmd.speed_percent;
        published_status.left_direction        = cmd.left_dir;
        published_status.right_direction       = cmd.right_dir;
        published_status.emergency_stop        = (cmd.speed_percent == 0 &&
                                                    cmd.left_dir  == WHEEL_STOP &&
                                                    cmd.right_dir == WHEEL_STOP);
        published_status.sequence              = ++command_sequence;
        published_status.timestamp_ms          = now_ms();
        pthread_mutex_unlock(&status_pub_lock);

        reply.accepted = 1;
        MsgReply(rcvid, EOK, &reply, sizeof(reply));

        clock_gettime(CLOCK_MONOTONIC, &done_time);
        stats_add(&exec_stats, elapsed_ms(&recv_time, &done_time));

        if (exec_stats.count >= STATS_WINDOW_TICKS)
        {
            logger_log("[MotorController][stats] period min/max/avg=%.2f/%.2f/%.2fms "
                   "execution min/max/avg=%.2f/%.2f/%.2fms (window=%d ticks)",
                   period_stats.min, period_stats.max,
                   period_stats.count ? period_stats.sum / period_stats.count : 0.0f,
                   exec_stats.min, exec_stats.max, exec_stats.sum / exec_stats.count,
                   exec_stats.count);

            pthread_mutex_lock(&stats_pub_lock);
            published_stats.period.min_ms      = period_stats.min;
            published_stats.period.max_ms      = period_stats.max;
            published_stats.period.avg_ms      = period_stats.count
                                                ? period_stats.sum / period_stats.count : 0.0f;
            published_stats.period.samples     = period_stats.count;
            published_stats.execution.min_ms   = exec_stats.min;
            published_stats.execution.max_ms   = exec_stats.max;
            published_stats.execution.avg_ms   = exec_stats.sum / exec_stats.count;
            published_stats.execution.samples  = exec_stats.count;
            pthread_mutex_unlock(&stats_pub_lock);

            stats_reset(&period_stats);
            stats_reset(&exec_stats);
        }
    }

    return NULL; /* unreachable */
}

void motor_controller_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_MOTOR_CONTROLLER;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, motor_controller_thread, NULL) != 0)
    {
        perror("motor_controller_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);

    pthread_mutex_lock(&init_lock);
    while (!channel_ready)
        pthread_cond_wait(&init_cond, &init_lock);
    pthread_mutex_unlock(&init_lock);
}

int motor_controller_get_chid(void)
{
    return chid;
}

void motor_controller_emergency_stop_now(void)
{
    drv8833_emergency_stop();
}

void motor_controller_get_stats(motor_controller_stats_t *out)
{
    pthread_mutex_lock(&stats_pub_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_pub_lock);
}

void motor_controller_get_status(motor_status_t *out)
{
    pthread_mutex_lock(&status_pub_lock);
    *out = published_status;
    pthread_mutex_unlock(&status_pub_lock);
}
