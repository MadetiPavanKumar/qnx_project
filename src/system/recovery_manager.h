#ifndef RECOVERY_MANAGER_H
#define RECOVERY_MANAGER_H

#include <stdatomic.h>
#include <stdint.h>
#include "../common/watchdog.h"

/* Liveness state machine for one module's heartbeat, as actually
   observed by the watchdog - distinct from a recovery_entry_t's
   sensor-recovery state. A single late pulse is MISSED, not DEAD;
   only HEARTBEAT_CONSECUTIVE_MISS_LIMIT consecutive misses (each
   WATCHDOG_CHECK_INTERVAL_MS apart) earns PERSISTENT_FAULT. */
typedef enum
{
    WATCHDOG_HEALTHY = 0,      /* pulse seen within its deadline */
    WATCHDOG_MISSED_DEADLINE,  /* late, but not yet confirmed dead */
    WATCHDOG_PERSISTENT_FAULT  /* confirmed dead: consecutive-miss limit hit */
} watchdog_state_t;

typedef struct
{
    watchdog_state_t state;
    uint64_t         age_ms;        /* time since last heartbeat, at query time */
    int              consecutive_misses;
} watchdog_status_t;

/* Snapshot of one module's current watchdog state - safe to call
   from any thread (CLI included). Returns 0 and fills *out if the
   module has ever sent a heartbeat; returns -1 (out untouched) if
   nothing has been heard from it yet. */
int recovery_manager_get_watchdog_status(module_id_t module, watchdog_status_t *out);

/* Starts Recovery Manager and blocks until its channel exists - same
   pattern as safety_supervisor_start(). Call this FIRST in main(),
   before any other task, since every other task needs to connect to
   this channel to send heartbeats. */
void recovery_manager_start(void);

int recovery_manager_get_chid(void);

/* Outcome of one recovery attempt, reported back by the OWNING task
   after it runs its own recover() function - see
   recovery_manager_submit_result() below. */
typedef enum
{
    RECOVERY_RESULT_SUCCESS = 0,
    RECOVERY_RESULT_FAILURE = 1,
    RECOVERY_RESULT_NONE    = 2   /* no attempt has completed yet */
} recovery_result_t;

/* Report that a named sensor/module has gone FAULT and needs
   recovery. Safe to call from any thread. Recovery Manager only
   schedules WHEN a retry is due (backoff/attempt-limit policy) - it
   never touches the driver itself. The owning task (e.g. Sensor
   Monitor) must poll recovery_manager_poll_pending() with this same
   name on its OWN thread, run its own deinit()/init()/verify()
   in-line when it returns 1, and report the result with
   recovery_manager_submit_result(). This keeps driver
   deinit/init/verify calls confined to the thread that owns the
   driver, per the sensor-ownership rule. On success,
   *out_recovered_flag is atomically set to 1 - the caller should poll
   this each tick and, once true, reset its OWN local health/filter
   state and clear the flag back to 0. */
void recovery_manager_report_fault(const char *name, atomic_int *out_recovered_flag);

/* Call once per loop tick, on the OWNING task's own thread, for each
   name previously registered via recovery_manager_report_fault().
   Returns 1 (edge-triggered, once per due attempt) when a recovery
   attempt is due right now - the caller must immediately run its own
   recover() and then call recovery_manager_submit_result(). Returns 0
   otherwise. Must NEVER be used to justify calling the driver from
   any thread other than the owner's. */
int recovery_manager_poll_pending(const char *name);

/* Reports the outcome of a recovery attempt the owner just ran
   in-line, after a recovery_manager_poll_pending() call returned 1.
   Safe to call only from the same thread/context that polled. */
void recovery_manager_submit_result(const char *name, recovery_result_t result);

/* External, read-only view of one name's recovery history - for the
   CLI/telemetry service (item 25). Distinct from the internal
   active/pending/in_progress bookkeeping above: this is "what would a
   dashboard show", collapsed to the state flow the project's spec
   describes. */
typedef enum
{
    RECOVERY_STATE_NONE = 0,          /* this name has never reported a fault */
    RECOVERY_STATE_SCHEDULED,         /* FAULT reported; next attempt scheduled but not due yet */
    RECOVERY_STATE_RUNNING,           /* an attempt is due or currently executing on the owner's thread */
    RECOVERY_STATE_RECOVERED,         /* most recent attempt succeeded; no fault outstanding */
    RECOVERY_STATE_PERSISTENT_FAULT   /* attempts exhausted (RECOVERY_MAX_ATTEMPTS); sensor still faulted */
} recovery_state_t;

typedef struct
{
    recovery_state_t   state;
    int                attempt_count;   /* attempts made in the current/most recent fault cycle */
    uint64_t           next_retry_ms;   /* CLOCK_MONOTONIC; meaningful only when state == RECOVERY_STATE_SCHEDULED */
    recovery_result_t  last_result;     /* outcome of the most recently completed attempt */
    int                failure_count;   /* cumulative failed attempts across this name's whole lifetime */
} recovery_status_t;

/* Safe to call from any thread (CLI, telemetry service). Returns 0
   and fills *out if `name` has ever been reported via
   recovery_manager_report_fault(); returns -1 (out untouched,
   equivalent to RECOVERY_STATE_NONE) if it never has. Never blocks -
   just a short, mutex-protected struct copy, same pattern as
   recovery_manager_get_watchdog_status() above. */
int recovery_manager_get_recovery_status(const char *name, recovery_status_t *out);

#endif /* RECOVERY_MANAGER_H */
