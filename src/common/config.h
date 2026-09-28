#ifndef CONFIG_H
#define CONFIG_H

/* ---- Timing ---------------------------------------------------------
   TICK_INTERVAL_MS is the core real-time parameter of the whole
   project: Navigation re-sends its intent this often, and Safety
   re-checks sensors this often. It directly sets your worst-case
   deadline bound (see safety_supervisor.c for how that's reported). */
#define TICK_INTERVAL_MS        75

/* Your real hc_sr04.h driver already paces itself: it won't return a
   fresh reading faster than once every ~65ms PER SENSOR (see
   HC_SR04_DEFAULT_MEASUREMENT_INTERVAL_US). Taking 5 samples inside
   one 75ms tick would mean blocking for ~325ms - it would break the
   tick model entirely. So SAMPLES_PER_TICK is 1: one real reading
   per sensor per tick, and the median/plausibility filtering in
   sensor_health.c works ACROSS ticks instead of within one - it
   still remembers the last good reading and still rejects
   implausible jumps, it just does it reading-by-reading over time
   rather than 5-at-once. No code changes needed there for this. */
#define SAMPLES_PER_TICK         1

/* ---- Ultrasonic distance thresholds (cm) ----------------------------
   > WARNING_DISTANCE_CM         -> full requested speed
   EMERGENCY..WARNING            -> speed scaled down linearly
   <= EMERGENCY_DISTANCE_CM      -> hard stop, regardless of request  */
#define WARNING_DISTANCE_CM     60.0f
#define EMERGENCY_DISTANCE_CM   15.0f

/* A reading that implies the object moved further than this between
   two ticks is treated as implausible (i.e. sensor noise), not acted
   on directly. Set generously above your real max speed's per-tick
   travel distance - tighten this once you've measured your robot's
   actual speed. */
#define MAX_PLAUSIBLE_DELTA_CM  120.0f

/* ---- Sensor health -----------------------------------------------
   Consecutive "bad" readings (timeout / implausible jump) before a
   sensor is downgraded. FAULT forces a safe stop independent of what
   the last good distance reading was. */
#define SENSOR_DEGRADED_AFTER    3
#define SENSOR_FAULT_AFTER       8

/* ---- Watchdog / Recovery Manager -----------------------------------
   Every task pulses Recovery Manager this often. Root-cause history:
   the false "Navigation/Safety/MotorController missed heartbeat"
   storm at ~350-390ms was NOT a watchdog-timeout problem - it was
   Safety Supervisor calling directly into OLED/I2C (status_display_
   update(), which does several devctl() calls) from its own
   real-time tick. devctl() on the I2C resource manager has no
   bounded timeout here, so any bus hiccup stalled Safety, which
   cascaded into Navigation's MsgSend() reply and Motor Controller's
   command both arriving late in the same tick - exactly the
   simultaneous 3-task "miss" pattern observed. Display/OLED now runs
   on its own low-priority task (display_task.c) that only ever reads
   telemetry_get() - Safety never blocks on it. These timeouts are
   therefore sized for a fast, non-blocking control loop, not padded
   to hide a stall. Do NOT inflate these to "fix" a future symptom -
   find the new blocking call instead. */
#define HEARTBEAT_INTERVAL_MS       TICK_INTERVAL_MS

/* Per-task heartbeat deadlines. Every task in the Navigation ->
   Safety -> Motor Controller chain does a small, bounded amount of
   work per tick now that OLED is off the critical path, so they
   share one base deadline; Sensor Monitor's driver reads are
   legitimately slower (see below) and gets its own. This is a
   deadline for ONE tick, not the "declare it dead" threshold - see
   HEARTBEAT_CONSECUTIVE_MISS_LIMIT below for that. */
#define NAVIGATION_HEARTBEAT_TIMEOUT_MS        (TICK_INTERVAL_MS * 2)
#define SAFETY_HEARTBEAT_TIMEOUT_MS            (TICK_INTERVAL_MS * 2)
#define MOTOR_CONTROLLER_HEARTBEAT_TIMEOUT_MS  (TICK_INTERVAL_MS * 2)

/* Backward-compatible alias - kept because it's a reasonable default
   for any module without a more specific constant above. */
#define HEARTBEAT_TIMEOUT_MS        (TICK_INTERVAL_MS * 4)

#define WATCHDOG_CHECK_INTERVAL_MS  100

/* A single missed deadline is not proof of death - it may just be
   legitimate scheduling jitter or a slightly slow tick. Recovery
   Manager only escalates to a logged "considered DEAD" / persistent-
   fault state after this many CONSECUTIVE watchdog-check intervals
   (each WATCHDOG_CHECK_INTERVAL_MS apart) have found the same task
   still over its deadline. This is the mechanism that replaces
   "one slow tick = false alarm" - not a bigger timeout number. */
#define HEARTBEAT_CONSECUTIVE_MISS_LIMIT   3

/* Sensor Monitor's cadence is NOT tied to TICK_INTERVAL_MS - it's
   however long a full ultrasonic + encoder + IMU read cycle actually
   takes on real hardware, which can legitimately exceed 300ms when a
   sensor is absent/timing out (ultrasonic echo timeouts, the encoder
   driver's ~100-iteration GPIO polling loop where each read is an
   IPC round-trip). Give it real headroom instead of sharing
   Navigation's tight timeout. */
#define SENSOR_MONITOR_HEARTBEAT_TIMEOUT_MS   800

/* A sensor snapshot older than this is no longer "fresh" - Safety
   must not keep trusting it forever just because it was once valid.
   Sized a bit above Sensor Monitor's own worst-case cycle time so a
   momentarily slow (not dead) read cycle isn't itself a fault. */
#define SENSOR_SNAPSHOT_STALE_MS    (SENSOR_MONITOR_HEARTBEAT_TIMEOUT_MS)

/* The safety decision itself (evaluate_safety() through replying to
   Navigation) must complete within one tick - this is the literally-
   enforced form of "override unsafe commands within a defined
   deadline," not just a reported number. If it doesn't, Safety fails
   safe on the spot: forces EMERGENCY/0% for that tick and logs a
   deadline-violation event, rather than approving a command computed
   from a decision that ran long. */
#define SAFETY_DEADLINE_MS          TICK_INTERVAL_MS

/* How often the aggregated min/max/avg timing statistics are printed
   from each real-time task, in ticks (not ms) - keeps the real-time
   threads from doing console I/O every single tick while still
   surfacing worst-case numbers regularly. */
#define STATS_WINDOW_TICKS          40

/* Per-tick console printf() from Safety/Navigation/Sensor Monitor is
   OFF by default - console I/O is not deterministic and must not run
   from a real-time thread on every 75ms tick (see PROJECT_SYSTEM_
   STATUS.md and the corrections doc). Flip this on only for manual
   debug sessions; the aggregated STATS_WINDOW_TICKS summaries and the
   transition-based event log stay on regardless. */
#define VERBOSE_TICK_LOGGING        0

/* How long Recovery Manager waits before attempting (or re-attempting)
   recovery of a faulted sensor - the "Recovery Timer." Deliberately
   NOT instant, so a flaky sensor doesn't get hammered with restarts. */
#define RECOVERY_RETRY_DELAY_MS     500
#define RECOVERY_MAX_ATTEMPTS       5

/* ---- Thread priorities (SCHED_FIFO) --------------------------------
   Higher number = higher priority, matching QNX/POSIX convention.
   Safety Supervisor must be able to preempt everything else - this is
   what makes "Priority Override" a real, enforced property.

   Sensor Monitor is DELIBERATELY lower than Navigation and Motor
   Controller, not higher - its drivers (HC-SR04, encoder) busy-poll
   with brief usleep()s rather than truly blocking, and giving it a
   higher priority than the control-loop threads let it repeatedly
   preempt and effectively starve Navigation (confirmed: Navigation's
   own tick rate collapsed to match Sensor Monitor's slow polling
   cycle when this was tried the other way around). Safety always
   works from whatever snapshot is latest regardless of how often
   Sensor Monitor gets scheduled, so this ordering costs nothing in
   correctness and fixes real starvation. */
#define PRIORITY_SAFETY_SUPERVISOR   30
#define PRIORITY_MOTOR_CONTROLLER    25
#define PRIORITY_NAVIGATION          20
#define PRIORITY_RECOVERY_MANAGER    15
#define PRIORITY_SENSOR_MONITOR      10

/* Display/Telemetry (OLED) - deliberately the lowest priority thread
   in the system. It only ever reads a mutex-protected telemetry
   snapshot and does slow I2C writes; it must never be able to
   preempt or meaningfully delay anything above it, and Safety must
   never wait on it (see safety_supervisor.c - it no longer calls
   into this task's code at all, only telemetry_update()). */
#define PRIORITY_DISPLAY              4

/* Recovery worker threads (the ones that actually call a driver's
   recover() function) run at their OWN low, explicit priority -
   without this, pthread_create(..., NULL, ...) defaults to INHERITING
   the creating thread's scheduling (Recovery Manager's priority 15),
   which is higher than it needs to be for background retry work. */
#define PRIORITY_RECOVERY_WORKER      5

/* Display task's own polling cadence - independent of and slower
   than TICK_INTERVAL_MS, since a human reading an OLED does not need
   128px of text redrawn 13 times a second, and this keeps I2C bus
   traffic (and any devctl() latency) off the critical path's cadence
   entirely. */
#define DISPLAY_UPDATE_INTERVAL_MS   150

/* ---- Logger (common/logger.c) --------------------------------------
   A dedicated, low-priority Logger task owns all actual terminal
   I/O. Every other task only ever does a bounded, O(1) push into this
   fixed-size ring buffer (see logger_log()/logger_log_err() in
   logger.c) and returns immediately - the potentially slow fputs()/
   fflush() to a real terminal happens entirely on the Logger task's
   own thread, never on a real-time task's. Sized generously above
   this project's actual log volume (console lines, human-readable
   pace) - see logger_get_dropped_count() for the overflow counter if
   this is ever too small for a debug session. */
#define LOGGER_QUEUE_CAPACITY        64
#define PRIORITY_LOGGER               3

/* ---- Telemetry Service (system/telemetry_service.c) ----------------
   The cloud-facing layer, built ON TOP OF telemetry.c's synchronous
   safety/sensor snapshot - see telemetry_service.h for the full
   design. Deliberately the lowest-priority SCHED_FIFO thread in the
   system (lower even than Display): a stalled network operation must
   never be able to delay anything above it, Display included. */
#define PRIORITY_TELEMETRY            2

/* How often a complete system snapshot is assembled for the cloud -
   item 14. Independent of TICK_INTERVAL_MS; the robot's own control
   loops keep running at their existing rate regardless of this. */
#define TELEMETRY_SNAPSHOT_INTERVAL_MS   1000

/* Bounded event queue awaiting upload - item 29. Small: this holds
   TRANSITION events only (state changes, faults, recoveries), not a
   log of every tick, so it should rarely come anywhere close to
   full in normal operation. */
#define TELEMETRY_EVENT_QUEUE_CAPACITY   64

/* Bounded exponential backoff for reconnect attempts once a transport
   is registered (Phase 4) - item 36. Not used by anything until a
   transport exists; the state machine has nowhere to retry yet
   without one. */
#define TELEMETRY_BACKOFF_INITIAL_MS     1000
#define TELEMETRY_BACKOFF_MAX_MS        30000

/* Network transport (system/telemetry_transport.c) - item 33/46.
   Every network operation has a finite timeout; only the Telemetry
   Service thread ever waits on one of these - no real-time task goes
   anywhere near this file. */
#define TELEMETRY_CONNECT_TIMEOUT_MS    3000
#define TELEMETRY_SEND_TIMEOUT_MS       3000
#define TELEMETRY_RECV_TIMEOUT_MS       5000
#define TELEMETRY_MAX_PAYLOAD_BYTES     8192  /* one snapshot's JSON body */
#define TELEMETRY_MAX_RESPONSE_BYTES    2048  /* HTTP response we bother reading */
#define TELEMETRY_EVENT_BATCH_MAX         16  /* queued events uploaded per cycle */

/* Identifies this physical robot in telemetry snapshots/events -
   item 16. A compile-time default; change per-unit if you ever have
   more than one of these on the same dashboard. */
#define ROBOT_ID  "qnx-rover-01"

/* ---- IMU / encoder safety thresholds -------------------------------
   TEST VALUES ONLY. These are reasonable starting guesses, not
   measured from your actual robot - tune every one of these against
   real IMU/encoder readings once the hardware is running (e.g. log
   accel magnitude while the robot sits still and while it takes a
   real hit, log RPM at your actual top speed) before trusting them
   for anything beyond bench testing. */

/* At rest, accel magnitude should read close to 1.0g. A sudden jolt/
   impact pushes this well above 1.0g momentarily. */
#define IMU_MAX_ACCEL_MAGNITUDE_G   1.8f

/* az close to 1.0g means upright; a low az means the chassis has
   tilted significantly (possible tip-over in progress). */
#define IMU_MIN_UPRIGHT_AZ_G        0.5f

/* Wheel RPM ceilings. */
#define ENCODER_WARNING_RPM         150.0f
#define ENCODER_MAX_RPM             200.0f

/* If one wheel spins this many times faster than the other while
   both are supposedly driven the same way, treat it as a possible
   stall/slip rather than trusting it. */
#define ENCODER_IMBALANCE_RATIO     2.0f

#endif /* CONFIG_H */
