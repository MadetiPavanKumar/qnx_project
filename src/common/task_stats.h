#ifndef TASK_STATS_H
#define TASK_STATS_H

/* Shared shape for the read-only latency snapshots exposed by
   safety_supervisor_get_stats(), sensor_monitor_get_stats(),
   navigation_get_stats(), motor_controller_get_stats() and
   display_get_stats().
 *
 * Each task already computes real min/max/avg over a
 * STATS_WINDOW_TICKS-sized window internally (see each .c file's own
 * local `stats_t` accumulator) and logs it once the window closes.
 * This struct is what gets PUBLISHED at that same moment: the task
 * copies its just-closed window's numbers into a small static
 * instance of this struct, protected by a tiny dedicated mutex, and
 * the getter copies it back out. Nothing here duplicates the
 * measurement itself - it's a snapshot of a number that already
 * exists, taken at the one point (window close) where it's a
 * complete, real window rather than a partially-accumulated one.
 *
 * `samples` is how many measurements the min/max/avg below were
 * computed from. 0 means no window has completed yet (e.g. right
 * after startup) - callers should treat min/max/avg as not-yet-
 * meaningful in that case rather than displaying stale zeros as if
 * they were real.
 */
typedef struct
{
    float min_ms;
    float max_ms;
    float avg_ms;
    int   samples;
} task_latency_stats_t;

#endif /* TASK_STATS_H */
