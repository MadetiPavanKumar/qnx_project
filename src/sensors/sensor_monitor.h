#ifndef SENSOR_MONITOR_H
#define SENSOR_MONITOR_H

#include "../common/task_stats.h"

void sensor_monitor_start(void);

/* Read-only latency snapshot - one group per phase Sensor Monitor
   times each cycle, same numbers the periodic "[SensorMonitor][stats]"
   log line already reports (see sensor_monitor.c). `cycle` is the
   whole-loop period; the other four are how long each sensor's own
   read took within that cycle. */
typedef struct
{
    task_latency_stats_t cycle;
    task_latency_stats_t ultrasonic;
    task_latency_stats_t ir;
    task_latency_stats_t encoder;
    task_latency_stats_t imu;
} sensor_monitor_stats_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy, refreshed once per STATS_WINDOW_TICKS
   (cycle.samples == 0 until the first window closes). */
void sensor_monitor_get_stats(sensor_monitor_stats_t *out);

#endif /* SENSOR_MONITOR_H */
