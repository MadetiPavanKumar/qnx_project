#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include "../common/task_stats.h"

/* Starts the Safety Supervisor thread and blocks until its QNX
   channel exists and is ready to receive messages. Call this before
   navigation_start(), since navigation needs the channel id below. */
void safety_supervisor_start(void);

/* Returns the chid of the Safety Supervisor's channel. Only valid
   after safety_supervisor_start() has returned. */
int safety_supervisor_get_chid(void);

/* Read-only latency snapshot, published once per completed
   STATS_WINDOW_TICKS window (same numbers the periodic
   "[Safety][stats]" log line already reports - see safety_
   supervisor.c). `cycle` is the tick-to-tick period (time between
   successive Navigation requests arriving); `processing` is the
   decision latency itself (the number Safety's real-time deadline is
   actually measured against). `deadline_ms` is the enforced bound
   (SAFETY_DEADLINE_MS); `last_deadline_violated` reflects only the
   MOST RECENT tick, not history - use event_log_get_recent() (see
   telemetry.h) for a record of past violations. */
typedef struct
{
    task_latency_stats_t cycle;
    task_latency_stats_t processing;
    int                  deadline_ms;
    int                  last_deadline_violated;
} safety_stats_t;

/* Safe to call from any thread. Never blocks - a short mutex-
   protected struct copy, refreshed once per STATS_WINDOW_TICKS
   (cycle.samples == 0 until the first window closes). */
void safety_supervisor_get_stats(safety_stats_t *out);

#endif /* SAFETY_SUPERVISOR_H */
