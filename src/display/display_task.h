#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "../common/task_stats.h"

/* Owns status_display_init()/status_display_update() (and therefore
   all OLED/I2C access) on its own thread, at the lowest priority in
   the system (PRIORITY_DISPLAY). It polls telemetry_get() - the same
   mutex-protected snapshot the CLI reads - on its own
   DISPLAY_UPDATE_INTERVAL_MS cadence.

   This is the fix for "OLED/I2C in the Safety real-time path": Safety
   Supervisor no longer calls into status_display.c at all. A slow or
   hung I2C bus can stall THIS thread indefinitely without blocking
   Safety, Navigation, or Motor Controller, and without starving the
   watchdog - PRIORITY_DISPLAY is low enough that it can never preempt
   or meaningfully delay anything above it.

   Call after recovery_manager_start() (any point before or after the
   control-loop tasks is fine - this task has no dependency on their
   channels, only on telemetry.c, which is always available). */
void display_task_start(void);

typedef struct
{
    task_latency_stats_t oled_update;
} display_stats_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy, refreshed once per STATS_WINDOW_TICKS
   (oled_update.samples == 0 until the first window closes, or if no
   OLED was found at startup - see status_display_init()). */
void display_get_stats(display_stats_t *out);

#endif /* DISPLAY_TASK_H */
