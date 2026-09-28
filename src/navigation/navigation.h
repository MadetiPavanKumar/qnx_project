#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "../common/task_stats.h"
#include "../common/system_types.h"

/* Starts the Navigation thread. Must be called after
   safety_supervisor_start(), since it connects to Safety's channel. */
void navigation_start(void);

typedef struct
{
    task_latency_stats_t period;      /* tick-to-tick loop period */
    task_latency_stats_t execution;   /* request-build + MsgSend/reply round trip */
} navigation_stats_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy, refreshed once per STATS_WINDOW_TICKS
   (period.samples == 0 until the first window closes). */
void navigation_get_stats(navigation_stats_t *out);

/* Navigation's most recent commanded intent and the outcome of
   sending it to Safety - a lightweight read-only snapshot, not a
   duplicate of any decision logic (decide_intent() in navigation.c
   remains the only place that decides what to do). `health` reflects
   whether the last MsgSend() to Safety succeeded. */
typedef enum { NAV_HEALTH_OK = 0, NAV_HEALTH_FAULT } navigation_health_t;

typedef struct
{
    navigation_command_t command;
    int                  requested_speed_percent;
    navigation_health_t  health;
} navigation_status_t;

void navigation_get_status(navigation_status_t *out);

#endif /* NAVIGATION_H */
