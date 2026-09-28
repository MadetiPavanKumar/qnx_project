#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>

#include "recovery_manager.h"
#include "../common/config.h"
#include "../common/logger.h"

#define MAX_RECOVERY_ENTRIES  8

typedef struct
{
    int              active;          /* slot in use */
    const char      *name;
    uint64_t         last_seen_ms;
    int              consecutive_misses;  /* consecutive WATCHDOG_CHECK_INTERVAL_MS
                                              ticks found still over deadline */
    watchdog_state_t state;
} heartbeat_entry_t;

typedef struct
{
    int              active;          /* tracked, pending, or in-progress recovery */
    int              pending;         /* attempt is due now, waiting for the OWNER
                                          to poll it and run recover() itself */
    int              in_progress;     /* owner has polled it and is running recover()
                                          on its own thread; waiting for the result */
    const char      *name;            /* stays set (slot is never freed) once a name
                                          has ever been reported, so
                                          recovery_manager_get_recovery_status() can
                                          still answer for it after the fault clears */
    atomic_int      *recovered_flag;
    uint64_t         next_attempt_ms;
    int              attempts;            /* attempts made in the CURRENT fault cycle */
    int              gave_up_reported;
    recovery_result_t last_result;        /* outcome of the most recently completed
                                              attempt (RECOVERY_RESULT_NONE if none yet) */
    int              failure_count;       /* cumulative failed attempts across this
                                              sensor's whole lifetime, never reset */
} recovery_entry_t;

static int chid = -1;
static pthread_mutex_t init_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  init_cond = PTHREAD_COND_INITIALIZER;
static int channel_ready = 0;

static pthread_mutex_t state_lock = PTHREAD_MUTEX_INITIALIZER;
static heartbeat_entry_t heartbeats[MODULE_COUNT];
static recovery_entry_t  recoveries[MAX_RECOVERY_ENTRIES];

static const char *module_name(module_id_t m)
{
    switch (m)
    {
        case MODULE_NAVIGATION:      return "Navigation";
        case MODULE_SAFETY:          return "Safety";
        case MODULE_SENSOR_MONITOR:  return "SensorMonitor";
        case MODULE_MOTOR_CONTROLLER: return "MotorController";
        default:                      return "?";
    }
}

/* Small fixed-size scratch buffer for messages produced while
   state_lock is held. Every state_lock-holding function below writes
   into one of these instead of calling logger_log_err() directly, so
   the actual console write (and the mutex logger.c uses for it)
   never happens while state_lock is held - the two mutexes are never
   nested in either order, anywhere in this file. Sized for the worst
   case of one line per module in a single watchdog tick. */
#define PENDING_LOG_LINE_MAX  160
typedef struct { char text[PENDING_LOG_LINE_MAX]; } pending_log_t;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

/* Called by Recovery Manager's own thread only (via the periodic
   timer pulse) - checks every module's last heartbeat. This DETECTS
   a hung/dead task; it does not attempt to restart one - restarting
   a QNX thread safely from outside itself is a much bigger problem
   than this project's scope. What it buys you: an honest, logged
   record that a task died, instead of silent hangs.

   Per-task deadline, NOT one universal timeout - a slow driver read
   cycle on Sensor Monitor is not the same kind of "late" as Safety
   missing its own tick. */
static uint64_t heartbeat_timeout_for(module_id_t m)
{
    switch (m)
    {
        case MODULE_SENSOR_MONITOR:   return SENSOR_MONITOR_HEARTBEAT_TIMEOUT_MS;
        case MODULE_NAVIGATION:       return NAVIGATION_HEARTBEAT_TIMEOUT_MS;
        case MODULE_SAFETY:           return SAFETY_HEARTBEAT_TIMEOUT_MS;
        case MODULE_MOTOR_CONTROLLER: return MOTOR_CONTROLLER_HEARTBEAT_TIMEOUT_MS;
        default:                      return HEARTBEAT_TIMEOUT_MS;
    }
}

/* State machine per module:

     HEALTHY --(1 late pulse)--> MISSED_DEADLINE --(N-1 more, still
     late)--> PERSISTENT_FAULT --(a pulse arrives within deadline
     again, from ANY state)--> HEALTHY

   A single late heartbeat is completely ordinary (scheduling jitter,
   a legitimately slightly slower tick) and is NOT logged as a
   failure - only the transition into PERSISTENT_FAULT is logged as
   "considered DEAD," and only after HEARTBEAT_CONSECUTIVE_MISS_LIMIT
   consecutive checks have found it still late. This is what
   distinguishes "heartbeat late" from "task actually dead" instead
   of treating the first missed tick as proof of death.

   Called with state_lock held. Writes any messages into `pending`
   (caller-provided, sized MODULE_COUNT) and bumps *pending_count
   instead of logging directly - the caller logs them after
   state_lock is released. */
static void check_heartbeats_locked(pending_log_t *pending, int *pending_count)
{
    uint64_t now = now_ms();

    for (int i = 0; i < MODULE_COUNT; i++)
    {
        if (!heartbeats[i].active) continue;

        heartbeat_entry_t *h = &heartbeats[i];
        uint64_t age = now - h->last_seen_ms;
        uint64_t timeout = heartbeat_timeout_for((module_id_t)i);

        if (age > timeout)
        {
            h->consecutive_misses++;

            if (h->consecutive_misses >= HEARTBEAT_CONSECUTIVE_MISS_LIMIT)
            {
                if (h->state != WATCHDOG_PERSISTENT_FAULT)
                {
                    snprintf(pending[*pending_count].text, PENDING_LOG_LINE_MAX,
                             "[RecoveryManager] WATCHDOG: %s missed "
                             "heartbeat %d consecutive checks in a row "
                             "(%llums since last seen, deadline %llums) "
                             "- task considered DEAD",
                             module_name((module_id_t)i), h->consecutive_misses,
                             (unsigned long long)age, (unsigned long long)timeout);
                    (*pending_count)++;
                    h->state = WATCHDOG_PERSISTENT_FAULT;
                }
            }
            else if (h->state == WATCHDOG_HEALTHY)
            {
                /* First miss - note it internally, but this is
                   ordinary jitter until proven otherwise, so no
                   alarm yet. */
                h->state = WATCHDOG_MISSED_DEADLINE;
            }
        }
        else
        {
            if (h->state == WATCHDOG_PERSISTENT_FAULT)
            {
                snprintf(pending[*pending_count].text, PENDING_LOG_LINE_MAX,
                         "[RecoveryManager] %s heartbeat resumed - "
                         "task no longer considered dead",
                         module_name((module_id_t)i));
                (*pending_count)++;
            }
            h->consecutive_misses = 0;
            h->state = WATCHDOG_HEALTHY;
        }
    }
}

/* Called from the watchdog thread's own loop - must stay fast, and
   must NEVER call a driver function itself. This only flips a
   per-entry "pending" flag when a retry is due; the OWNING task
   (Sensor Monitor) picks that up via recovery_manager_poll_pending()
   on ITS OWN thread and runs the actual deinit()/init()/verify()
   there. Recovery Manager previously ran recover() itself (first
   directly, later via a detached worker thread it spawned) - both
   forms still let a thread that does not own the driver call
   deinit()/init() concurrently with Sensor Monitor's own reads
   (hcsr04_driver_deinit()'s pthread_mutex_destroy()/
   munmap_device_memory() racing hcsr04_read_raw_cm()), which is
   exactly the SIGSEGV/exit-139 ownership violation this project
   requires fixed. Scheduling and execution are now fully split:
   this function only decides WHEN, never WHO calls the driver. */
static void run_recovery_attempts_locked(void)
{
    uint64_t now = now_ms();

    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        recovery_entry_t *r = &recoveries[i];
        if (!r->active || r->pending || r->in_progress) continue;
        if (now < r->next_attempt_ms) continue;

        r->pending = 1;
    }
}

static void *recovery_manager_thread(void *arg)
{
    (void)arg;

    chid = ChannelCreate(0);
    if (chid == -1)
    {
        perror("recovery_manager: ChannelCreate failed");
        exit(EXIT_FAILURE);
    }

    pthread_mutex_lock(&init_lock);
    channel_ready = 1;
    pthread_cond_signal(&init_cond);
    pthread_mutex_unlock(&init_lock);

    /* Periodic self-pulse: a QNX timer that fires every
       WATCHDOG_CHECK_INTERVAL_MS and delivers a pulse to our own
       channel, so the same MsgReceive() loop below handles both
       heartbeats FROM other tasks and our own periodic check. */
    struct sigevent sev;
    int self_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    SIGEV_PULSE_INIT(&sev, self_coid, SIGEV_PULSE_PRIO_INHERIT,
                      PULSE_CODE_TIMER_TICK, 0);

    timer_t timer_id;
    timer_create(CLOCK_MONOTONIC, &sev, &timer_id);

    struct itimerspec its;
    its.it_value.tv_sec     = WATCHDOG_CHECK_INTERVAL_MS / 1000;
    its.it_value.tv_nsec    = (WATCHDOG_CHECK_INTERVAL_MS % 1000) * 1000000L;
    its.it_interval         = its.it_value;
    timer_settime(timer_id, 0, &its, NULL);

    for (;;)
    {
        struct _pulse pulse;
        int rcvid = MsgReceive(chid, &pulse, sizeof(pulse), NULL);

        if (rcvid != 0) continue; /* we only expect pulses here */

        pending_log_t pending[MODULE_COUNT];
        int pending_count = 0;

        pthread_mutex_lock(&state_lock);

        if (pulse.code == PULSE_CODE_HEARTBEAT)
        {
            module_id_t who = (module_id_t)pulse.value.sival_int;
            if (who >= 0 && who < MODULE_COUNT)
            {
                heartbeats[who].active       = 1;
                heartbeats[who].name         = module_name(who);
                heartbeats[who].last_seen_ms = now_ms();
            }
        }
        else if (pulse.code == PULSE_CODE_TIMER_TICK)
        {
            check_heartbeats_locked(pending, &pending_count);
            run_recovery_attempts_locked();
        }

        pthread_mutex_unlock(&state_lock);

        /* Logged AFTER releasing state_lock - logger.c's own mutex is
           never acquired while state_lock is held, in either order,
           anywhere in this file. */
        for (int i = 0; i < pending_count; i++)
        {
            logger_log_err("%s", pending[i].text);
        }
    }

    return NULL; /* unreachable */
}

void recovery_manager_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_RECOVERY_MANAGER;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, recovery_manager_thread, NULL) != 0)
    {
        perror("recovery_manager_start: pthread_create failed");
        exit(EXIT_FAILURE);
    }
    pthread_detach(tid);

    pthread_mutex_lock(&init_lock);
    while (!channel_ready)
        pthread_cond_wait(&init_cond, &init_lock);
    pthread_mutex_unlock(&init_lock);
}

int recovery_manager_get_chid(void)
{
    return chid;
}

int recovery_manager_get_watchdog_status(module_id_t module, watchdog_status_t *out)
{
    if (module < 0 || module >= MODULE_COUNT) return -1;

    pthread_mutex_lock(&state_lock);

    if (!heartbeats[module].active)
    {
        pthread_mutex_unlock(&state_lock);
        return -1;
    }

    out->state              = heartbeats[module].state;
    out->age_ms             = now_ms() - heartbeats[module].last_seen_ms;
    out->consecutive_misses = heartbeats[module].consecutive_misses;

    pthread_mutex_unlock(&state_lock);
    return 0;
}

int recovery_manager_get_recovery_status(const char *name, recovery_status_t *out)
{
    pthread_mutex_lock(&state_lock);

    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        recovery_entry_t *r = &recoveries[i];
        if (!r->name || strcmp(r->name, name) != 0) continue;

        if (!r->active)
        {
            out->state = (r->last_result == RECOVERY_RESULT_SUCCESS || r->last_result == RECOVERY_RESULT_NONE)
                       ? RECOVERY_STATE_RECOVERED
                       : RECOVERY_STATE_PERSISTENT_FAULT;
        }
        else if (r->pending || r->in_progress)
        {
            out->state = RECOVERY_STATE_RUNNING;
        }
        else
        {
            out->state = RECOVERY_STATE_SCHEDULED;
        }

        out->attempt_count    = r->attempts;
        out->next_retry_ms    = r->next_attempt_ms;
        out->last_result      = r->last_result;
        out->failure_count    = r->failure_count;

        pthread_mutex_unlock(&state_lock);
        return 0;
    }

    pthread_mutex_unlock(&state_lock);
    return -1; /* this name has never reported a fault - treat as RECOVERY_STATE_NONE */
}

void recovery_manager_report_fault(const char *name, atomic_int *out_recovered_flag)
{
    char msg[PENDING_LOG_LINE_MAX];
    int  should_log = 0;

    pthread_mutex_lock(&state_lock);

    /* Already actively tracking this one? Don't duplicate. */
    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        if (recoveries[i].name && recoveries[i].active &&
            strcmp(recoveries[i].name, name) == 0)
        {
            pthread_mutex_unlock(&state_lock);
            return;
        }
    }

    /* Reuse this name's own slot if it has one from a PREVIOUS fault
       cycle (slots are never freed once a name has been seen, so
       recovery_manager_get_recovery_status() can keep answering for
       it after the fault clears - see recovery_entry_t's `name`
       comment) - this also preserves failure_count across cycles,
       since that field is a lifetime total, not per-cycle. */
    int slot = -1;
    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        if (recoveries[i].name && strcmp(recoveries[i].name, name) == 0)
        {
            slot = i;
            break;
        }
    }
    if (slot == -1)
    {
        for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
        {
            if (recoveries[i].name == NULL) { slot = i; break; }
        }
    }

    if (slot != -1)
    {
        int is_fresh_slot = (recoveries[slot].name == NULL);

        recoveries[slot].active            = 1;
        recoveries[slot].pending           = 0;
        recoveries[slot].in_progress       = 0;
        recoveries[slot].name              = name;
        recoveries[slot].recovered_flag    = out_recovered_flag;
        recoveries[slot].attempts          = 0;
        recoveries[slot].gave_up_reported  = 0;
        recoveries[slot].next_attempt_ms   = now_ms() + RECOVERY_RETRY_DELAY_MS;
        if (is_fresh_slot)
        {
            /* Only initialize these the very first time this name is
               ever seen - on a REUSED slot (a later fault cycle for a
               name we've recovered before) they must keep carrying
               lifetime history, not reset to "no attempts yet". */
            recoveries[slot].last_result   = RECOVERY_RESULT_NONE;
            recoveries[slot].failure_count = 0;
        }

        snprintf(msg, sizeof(msg), "[RecoveryManager] %s reported FAULT - "
                                    "recovery attempt scheduled in %dms",
                 name, RECOVERY_RETRY_DELAY_MS);
        should_log = 1;
    }

    pthread_mutex_unlock(&state_lock);

    /* Logged after releasing state_lock - see the note in
       recovery_manager_thread() above. */
    if (should_log) logger_log_err("%s", msg);
}

int recovery_manager_poll_pending(const char *name)
{
    int due = 0;

    pthread_mutex_lock(&state_lock);

    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        recovery_entry_t *r = &recoveries[i];
        if (r->active && r->pending && strcmp(r->name, name) == 0)
        {
            r->pending     = 0;
            r->in_progress = 1;   /* now waiting on the owner's result */
            due = 1;
            break;
        }
    }

    pthread_mutex_unlock(&state_lock);
    return due;
}

void recovery_manager_submit_result(const char *name, recovery_result_t result)
{
    char msg[PENDING_LOG_LINE_MAX];
    int  should_log = 0;

    pthread_mutex_lock(&state_lock);

    for (int i = 0; i < MAX_RECOVERY_ENTRIES; i++)
    {
        recovery_entry_t *r = &recoveries[i];
        if (!(r->active && r->in_progress && strcmp(r->name, name) == 0)) continue;

        r->attempts++;

        if (result == RECOVERY_RESULT_SUCCESS)
        {
            snprintf(msg, sizeof(msg), "[RecoveryManager] %s RECOVERED after "
                                        "%d attempt(s)", r->name, r->attempts);
            should_log = 1;
            atomic_store(r->recovered_flag, 1);
            r->last_result = RECOVERY_RESULT_SUCCESS;
            r->active = 0;
        }
        else if (r->attempts >= RECOVERY_MAX_ATTEMPTS)
        {
            r->failure_count++;
            r->last_result = RECOVERY_RESULT_FAILURE;
            if (!r->gave_up_reported)
            {
                snprintf(msg, sizeof(msg), "[RecoveryManager] %s recovery "
                                            "ABANDONED after %d failed attempts "
                                            "- persistent FAULT",
                         r->name, r->attempts);
                should_log = 1;
                r->gave_up_reported = 1;
            }
            r->active = 0;
        }
        else
        {
            r->failure_count++;
            r->last_result = RECOVERY_RESULT_FAILURE;
            snprintf(msg, sizeof(msg), "[RecoveryManager] %s recovery attempt "
                                        "%d failed, retrying in %dms",
                     r->name, r->attempts, RECOVERY_RETRY_DELAY_MS);
            should_log = 1;
            r->next_attempt_ms = now_ms() + RECOVERY_RETRY_DELAY_MS;
        }

        r->in_progress = 0;
        break;
    }

    pthread_mutex_unlock(&state_lock);

    /* Logged after releasing state_lock - see the note in
       recovery_manager_thread() above. */
    if (should_log) logger_log_err("%s", msg);
}
