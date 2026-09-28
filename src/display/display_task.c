#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include "display_task.h"
#include "status_display.h"
#include "../system/telemetry.h"
#include "../common/config.h"
#include "../common/logger.h"

static pthread_mutex_t stats_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static display_stats_t published_stats;

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

/* Simple running min/max/avg over a fixed window, printed
   periodically instead of on every update - see item 5/13 of the
   corrections: measure OLED latency, but don't spam the console. */
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

static void *display_thread(void *arg)
{
    (void)arg;

    int have_oled = (status_display_init() == 0);

    stats_t oled_stats;
    stats_reset(&oled_stats);

    for (;;)
    {
        sleep_ms(DISPLAY_UPDATE_INTERVAL_MS);

        telemetry_t t;
        telemetry_get(&t);

        if (have_oled)
        {
            struct timespec s, e;
            clock_gettime(CLOCK_MONOTONIC, &s);
            status_display_update(&t);
            clock_gettime(CLOCK_MONOTONIC, &e);
            stats_add(&oled_stats, elapsed_ms(&s, &e));
        }

        if (oled_stats.count >= STATS_WINDOW_TICKS)
        {
            /* Routed through the centralized logger rather than a
               direct fprintf(), same as every other task's periodic
               stats line (item 6) - Display runs at the lowest
               priority in the system, but it's still a thread other
               than the CLI writing to stdout, so it goes through the
               same serialization point as everything else to avoid
               tearing whatever the CLI has on screen. */
            logger_log("[Display] OLED update min/max/avg = "
                       "%.2f/%.2f/%.2fms over %d updates",
                       oled_stats.min, oled_stats.max,
                       oled_stats.sum / oled_stats.count, oled_stats.count);

            pthread_mutex_lock(&stats_pub_lock);
            published_stats.oled_update.min_ms  = oled_stats.min;
            published_stats.oled_update.max_ms  = oled_stats.max;
            published_stats.oled_update.avg_ms  = oled_stats.sum / oled_stats.count;
            published_stats.oled_update.samples = oled_stats.count;
            pthread_mutex_unlock(&stats_pub_lock);

            stats_reset(&oled_stats);
        }
    }

    return NULL; /* unreachable */
}

void display_task_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_DISPLAY;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, display_thread, NULL) != 0)
    {
        perror("display_task_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);
}

void display_get_stats(display_stats_t *out)
{
    pthread_mutex_lock(&stats_pub_lock);
    *out = published_stats;
    pthread_mutex_unlock(&stats_pub_lock);
}
