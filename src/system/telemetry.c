#include <stdio.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include "telemetry.h"

#define EVENT_LOG_CAPACITY 64

static pthread_mutex_t telemetry_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_t current;

static pthread_mutex_t event_lock = PTHREAD_MUTEX_INITIALIZER;
static event_entry_t   event_ring[EVENT_LOG_CAPACITY];
static int             event_head  = 0;  /* next slot to write */
static int             event_count = 0;  /* entries used, up to capacity */

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

void telemetry_update(const telemetry_t *snapshot)
{
    pthread_mutex_lock(&telemetry_lock);
    current = *snapshot;
    current.last_update_ms = now_ms();
    pthread_mutex_unlock(&telemetry_lock);
}

void telemetry_get(telemetry_t *out)
{
    pthread_mutex_lock(&telemetry_lock);
    *out = current;
    pthread_mutex_unlock(&telemetry_lock);
}

void event_log_add(const char *fmt, ...)
{
    pthread_mutex_lock(&event_lock);

    event_entry_t *slot = &event_ring[event_head];
    slot->time_ms = now_ms();

    va_list args;
    va_start(args, fmt);
    vsnprintf(slot->message, EVENT_MESSAGE_MAX, fmt, args);
    va_end(args);

    event_head = (event_head + 1) % EVENT_LOG_CAPACITY;
    if (event_count < EVENT_LOG_CAPACITY) event_count++;

    pthread_mutex_unlock(&event_lock);
}

int event_log_get_recent(event_entry_t *out, int max_count)
{
    pthread_mutex_lock(&event_lock);

    int n = event_count < max_count ? event_count : max_count;
    for (int i = 0; i < n; i++)
    {
        /* Walk backwards from the most recently written slot. */
        int idx = (event_head - 1 - i + EVENT_LOG_CAPACITY) % EVENT_LOG_CAPACITY;
        out[i] = event_ring[idx];
    }

    pthread_mutex_unlock(&event_lock);
    return n;
}
