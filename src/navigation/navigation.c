#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sys/neutrino.h>

#include "navigation.h"
#include "../safety/safety_supervisor.h"
#include "../common/messages.h"
#include "../common/config.h"
#include "../common/watchdog.h"
#include "../common/logger.h"
#include "../system/recovery_manager.h"

static void decide_intent(navigation_request_t *out)
{
    out->command = CMD_FORWARD;
    out->requested_speed_percent = 80;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static float elapsed_ms(struct timespec *start, struct timespec *end)
{
    float sec_part  = (float)(end->tv_sec - start->tv_sec) * 1000.0f;
    float nsec_part = (float)(end->tv_nsec - start->tv_nsec) / 1000000.0f;
    return sec_part + nsec_part;
}

/* Simple running min/max/avg over a fixed window - see
   STATS_WINDOW_TICKS in config.h. */
typedef struct { float sum, min, max; int count; } stats_t;
static void stats_reset(stats_t *s) { s->sum = 0; s->min = 1.0e9f; s->max = 0; s->count = 0; }
static void stats_add(stats_t *s, float v)
{
    s->sum += v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->count++;
}

/* Published snapshots - see navigation_get_stats()/navigation_get_status()
   in the header. Two independent, tiny mutexes rather than one: status
   changes every tick (cheap, high frequency), stats only once per
   window - no reason to make a status reader wait behind a stats
   publish or vice versa. Neither is ever nested with the other or
   with any other subsystem's lock. */
static pthread_mutex_t stats_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static navigation_stats_t published_stats;

static pthread_mutex_t status_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static navigation_status_t published_status;

static void *navigation_thread(void *arg)
{
    (void)arg;

    int safety_chid = safety_supervisor_get_chid();
    int coid = ConnectAttach(ND_LOCAL_NODE, getpid(), safety_chid,
                              _NTO_SIDE_CHANNEL, 0);
    if (coid == -1)
    {
        perror("navigation: ConnectAttach failed");
        exit(EXIT_FAILURE);
    }

    int rm_coid = ConnectAttach(ND_LOCAL_NODE, 0, recovery_manager_get_chid(),
                                  _NTO_SIDE_CHANNEL, 0);

    safety_message_t request;
    safety_reply_t   reply;
    request.type = MSG_NAVIGATION;

    stats_t period_stats, exec_stats;
    stats_reset(&period_stats);
    stats_reset(&exec_stats);
    struct timespec prev_loop_start;
    int have_prev_loop_start = 0;

    for (;;)
    {
        struct timespec loop_start, send_done;
        clock_gettime(CLOCK_MONOTONIC, &loop_start);

        if (have_prev_loop_start)
            stats_add(&period_stats, elapsed_ms(&prev_loop_start, &loop_start));
        prev_loop_start = loop_start;
        have_prev_loop_start = 1;

        /* Heartbeat is sent HERE - at the top of the loop, before the
           blocking MsgSend() to Safety - not after the reply comes
           back. This is the fix for item 1 (heartbeat/IPC coupling):
           Navigation's pulse now proves "this thread woke up and is
           executing its own loop," independent of how long Safety
           takes to reply. If Safety (or the whole chain) genuinely
           hangs, Navigation stops reaching this line on the next
           iteration and the watchdog still (correctly, eventually)
           notices - this only removes the FALSE alarm from ordinary,
           bounded IPC wait time. */
        MsgSendPulse(rm_coid, -1, PULSE_CODE_HEARTBEAT, MODULE_NAVIGATION);

        decide_intent(&request.navigation);

        int send_ok = (MsgSend(coid, &request, sizeof(request), &reply, sizeof(reply)) != -1);

        pthread_mutex_lock(&status_pub_lock);
        published_status.command                 = request.navigation.command;
        published_status.requested_speed_percent = request.navigation.requested_speed_percent;
        published_status.health = send_ok ? NAV_HEALTH_OK : NAV_HEALTH_FAULT;
        pthread_mutex_unlock(&status_pub_lock);

        if (!send_ok)
        {
            perror("navigation: MsgSend failed");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &send_done);
        stats_add(&exec_stats, elapsed_ms(&loop_start, &send_done));

        if (VERBOSE_TICK_LOGGING)
        {
            logger_log("[Navigation] requested=%d%% -> approved=%d%% state=%d",
                   request.navigation.requested_speed_percent,
                   reply.approved_speed_percent,
                   reply.state);
        }

        if (exec_stats.count >= STATS_WINDOW_TICKS)
        {
            logger_log("[Navigation][stats] period min/max/avg=%.2f/%.2f/%.2fms "
                   "execution(request+reply) min/max/avg=%.2f/%.2f/%.2fms "
                   "(window=%d ticks)",
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

        sleep_ms(TICK_INTERVAL_MS);
    }

    ConnectDetach(coid);
    return NULL;
}

void navigation_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_NAVIGATION;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, navigation_thread, NULL) != 0)
    {
        perror("navigation_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);
}

void navigation_get_stats(navigation_stats_t *out)
{
    pthread_mutex_lock(&stats_pub_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_pub_lock);
}

void navigation_get_status(navigation_status_t *out)
{
    pthread_mutex_lock(&status_pub_lock);
    *out = published_status;
    pthread_mutex_unlock(&status_pub_lock);
}
