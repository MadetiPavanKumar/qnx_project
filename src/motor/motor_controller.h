#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "../common/task_stats.h"
#include "../common/system_types.h"

void motor_controller_start(void);
int  motor_controller_get_chid(void);

/* Bypasses message passing entirely and calls drv8833_emergency_stop()
   directly - safe to call from ANY thread (that function is
   documented lock-free), including the CLI thread. Use for an
   immediate, out-of-band stop; the message-passing path (Safety ->
   Motor Controller) is still the normal control flow for everything
   else. */
void motor_controller_emergency_stop_now(void);

typedef struct
{
    task_latency_stats_t period;      /* tick-to-tick period between commands received */
    task_latency_stats_t execution;   /* time to execute one command (drv8833_set_motion() + reply) */
} motor_controller_stats_t;

/* Safe to call from any thread. Never blocks - short mutex-protected
   struct copy, refreshed once per STATS_WINDOW_TICKS
   (period.samples == 0 until the first window closes). */
void motor_controller_get_stats(motor_controller_stats_t *out);

/* Motor Controller's most recently EXECUTED command - a read-only
   snapshot of what it actually told the drv8833 to do (Motor
   Controller only ever executes what Safety already approved; this
   is not a second decision path). `sequence` increments once per
   command received - a telemetry consumer can tell from gaps in this
   number whether any commands were missed between two snapshots.
   `timestamp_ms` is CLOCK_MONOTONIC, matching every other timestamp
   in this project. */
typedef struct
{
    unsigned int  requested_motor_speed;   /* == approved_motor_speed: Motor
                                               Controller executes verbatim,
                                               it does not re-decide speed */
    unsigned int  approved_motor_speed;
    wheel_dir_t   left_direction;
    wheel_dir_t   right_direction;
    int           emergency_stop;          /* 1 if the last command was a full stop */
    uint64_t      sequence;
    uint64_t      timestamp_ms;
} motor_status_t;

void motor_controller_get_status(motor_status_t *out);

#endif /* MOTOR_CONTROLLER_H */
