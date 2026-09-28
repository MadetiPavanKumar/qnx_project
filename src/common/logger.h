#ifndef LOGGER_H
#define LOGGER_H

#include <stddef.h> /* size_t, for logger_raw_echo() */
#include <stdint.h> /* uint64_t, for logger_get_dropped_count() */

/* ============================================================
 * CENTRALIZED CONSOLE LOGGER
 * ============================================================
 *
 * Problem this solves: Navigation, Safety, SensorMonitor,
 * MotorController and RecoveryManager each print their own
 * diagnostics with raw printf()/fprintf() from their own threads,
 * completely independently of the interactive CLI thread that is
 * simultaneously doing printf("> ")/fgets(stdin). Two threads
 * writing to the same terminal at once (one printing a background
 * line, the other mid-way through drawing the "> " prompt) produces
 * torn/interleaved output like:
 *
 *     > st[Safety] dist=...
 *     atus
 *
 * This is a DISPLAY problem only - fgets() still receives whatever
 * the user actually typed correctly, since terminal echo and our
 * own writes are independent streams into the same fd. But it is
 * confusing and unacceptable for a CLI.
 *
 * Fix: every task funnels its console output through this module
 * instead of calling printf()/fprintf() directly. A single mutex
 * guarantees:
 *
 *   1. Two background lines from different tasks can never interleave
 *      with each other (each call prints one complete, already-
 *      formatted block atomically).
 *   2. A background line arriving while the CLI's "> " prompt is on
 *      screen moves to its own new line first, then reprints "> "
 *      afterwards - so the prompt is never split across a log line,
 *      and the user always ends up looking at a clean, freshly drawn
 *      prompt rather than a torn one.
 *   3. The CLI's own multi-line command output (status/watchdog/
 *      events) is likewise printed as one atomic block, so a
 *      concurrent background line can't land in the middle of it.
 *
 * Design constraints this module honors (see the individual task
 * files for how each call site respects them):
 *
 *   - vsnprintf() formats into a small stack buffer BEFORE any lock
 *     is taken. logger_log()/logger_log_err()/logger_debug()/
 *     logger_warn() then do a bounded, O(1) push of that already-
 *     formatted line into a fixed-size ring buffer (see
 *     LOGGER_QUEUE_CAPACITY in config.h) and return immediately - a
 *     dedicated, low-priority Logger task (started by logger_start(),
 *     see below) is the ONLY thread that ever does the actual
 *     fputs()/fflush() to the terminal. This means a real-time
 *     caller's logging call can never block on slow/stalled terminal
 *     I/O, and log_mutex (which guards the terminal itself, below) is
 *     never touched by a real-time task at all - only by the Logger
 *     task and the CLI thread (items 7/8/53).
 *   - If the ring buffer is ever full when a real-time task logs
 *     something, the single OLDEST queued entry is inspected (O(1),
 *     never a full scan): if it's DEBUG-level, it's discarded to make
 *     room; otherwise the INCOMING entry is what gets dropped. Either
 *     way logger_get_dropped_count() increments - see enqueue() in
 *     logger.c. DEBUG logging is filtered out before it's ever queued
 *     by default (see logger_set_level() below / VERBOSE_TICK_LOGGING
 *     in config.h), so in normal operation this queue should rarely
 *     if ever contain a DEBUG entry to evict - this policy is a
 *     defensive backstop, not something exercised every overflow.
 *   - log_mutex (guarding the terminal + prompt state) is never
 *     nested with any other lock in the project, and is never held
 *     across a sensor, IPC, I2C, SPI, GPIO, or safety-decision call.
 */

/* Starts the dedicated Logger task - call this FIRST in main(),
   before any other task starts (recovery_manager_start() included,
   since it logs too). Priority PRIORITY_LOGGER (config.h) - low, but
   still SCHED_FIFO so it isn't starved outright by a busy interactive
   shell; well below every real-time control task and Display. */
void logger_start(void);

typedef enum
{
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_ERROR = 3
} log_level_t;

/* Messages below this level are filtered out BEFORE being queued (not
   counted as dropped - this is deliberate, not overflow). Defaults to
   LOG_LEVEL_INFO - DEBUG stays off by default, matching the existing
   VERBOSE_TICK_LOGGING convention (item 50). Safe to call from any
   thread; intended for the CLI's "log level <level>" command. */
void logger_set_level(log_level_t level);
log_level_t logger_get_level(void);

/* Total log lines dropped due to the queue being full (see the
   overflow policy above) since startup - feeds
   telemetry_get_statistics()'s dropped_logs field (item 27). */
uint64_t logger_get_dropped_count(void);

/* Format and enqueue one INFO-level line for the Logger task to
   print to stdout. A trailing newline is added if the formatted text
   doesn't already end in one. Safe to call from any thread, including
   high-priority real-time tasks - formatting happens before anything
   is queued, and the actual terminal write happens later, on the
   Logger task's own thread (see the design note above). */
void logger_log(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/* Same as logger_log(), but ERROR-level and enqueued for stderr -
   used for the diagnostic/recovery messages that were previously
   fprintf(stderr, ...) directly, so they stay on their own stream but
   are still serialized (by the Logger task) against stdout's prompt
   handling. */
void logger_log_err(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/* DEBUG-level (stdout) - filtered out by default, see
   logger_set_level() above. For high-frequency/verbose diagnostics
   that shouldn't print by default (item 50/51) - VERBOSE_TICK_LOGGING
   call sites are good candidates to route through this instead of a
   config.h on/off #if in a future pass, though that migration isn't
   required by this change. */
void logger_debug(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/* WARN-level (stdout). */
void logger_warn(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/* Prints an already-fully-formatted block of text (may contain
   embedded newlines, e.g. a multi-line "status" report) atomically,
   the same way logger_log() prints a single line. Intended for the
   CLI's own command output: gather all the data you need FIRST
   (telemetry_get(), recovery_manager_get_watchdog_status(), etc. -
   each of those takes and releases its own lock internally), build
   the complete text into one buffer, then call this once. That
   ordering means this module's mutex is never held while any other
   subsystem's lock is also held. */
void logger_print_block(const char *text);

/* Called by the CLI right after it prints "> " and before it blocks
   on fgets()/its raw-mode reader. While the prompt is marked visible,
   any logger_log()/logger_log_err()/logger_print_block() call from
   another thread will move to a new line before printing and
   reprint "> " followed by whatever the user has typed so far (see
   logger_input_set() below) - so neither the prompt nor a partially
   typed command is ever torn, and the user's in-progress input is
   never visually lost. */
void logger_show_prompt(const char *prompt);

/* Called by the CLI's raw-mode reader (see cli.c) after every
   keystroke that changes the current input line (character typed,
   backspace, etc.), with the line's current contents so far
   (null-terminated, no trailing newline). Cheap - just a snprintf()
   into a small static buffer under the same short-lived mutex
   logger_log() already uses; safe to call from the CLI thread at
   human typing speed. Only meaningful between a logger_show_prompt()
   and the matching logger_clear_prompt_visible() - ignored/harmless
   otherwise. If the CLI is falling back to plain fgets() (stdin is
   not a tty - see cli.c), it simply never calls this, and a
   background log line during that blocking fgets() still gets the
   pre-existing "erase and redraw '> '" behavior (fgets() itself is
   reading from the tty's own line buffer, which our process cannot
   see into until Enter is pressed, so there is no in-progress text
   to redraw in that fallback mode anyway). */
void logger_input_set(const char *current_line);

/* Writes exactly `len` raw bytes to stdout (e.g. one echoed
   keystroke, or a backspace's "\b \b" erase sequence) atomically
   with respect to every other logger_* call - so the CLI's own
   character-at-a-time echo (used only when raw terminal mode is
   active - see cli.c) can never be torn by a background log line
   landing mid-character-sequence either. Does no formatting and adds
   no newline; the caller controls exactly what bytes appear. */
void logger_raw_echo(const char *bytes, size_t len);

/* Called by the CLI as soon as fgets()/its raw-mode reader returns
   (i.e. the prompt line has been consumed/submitted), so background
   output printed while the CLI is busy processing a command is not
   mistaken for output printed while a prompt was still on screen. */
void logger_clear_prompt_visible(void);

#endif /* LOGGER_H */
