#ifndef WATCHDOG_H
#define WATCHDOG_H

/* Shared IPC contract between every task and Recovery Manager. */

typedef enum
{
    MODULE_NAVIGATION = 0,
    MODULE_SAFETY,
    MODULE_SENSOR_MONITOR,
    MODULE_MOTOR_CONTROLLER,
    MODULE_COUNT
} module_id_t;

/* Pulse code used for heartbeats. Recovery Manager tells them apart
   from its own internal periodic-check pulse (see recovery_manager.c)
   by code, not by which channel they arrived on - both share one
   channel. The pulse's "value" field carries the module_id_t. */
#define PULSE_CODE_HEARTBEAT    1
#define PULSE_CODE_TIMER_TICK   2

#endif /* WATCHDOG_H */
