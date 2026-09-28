#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>

#include "cli.h"
#include "telemetry.h"
#include "telemetry_service.h"
#include "recovery_manager.h"
#include "../common/labels.h"
#include "../common/watchdog.h"
#include "../common/logger.h"
#include "../motor/motor_controller.h"

static atomic_int estop_active = 0;

int cli_is_estop_active(void)
{
    return atomic_load(&estop_active);
}

/* Appends formatted text to buf[*pos .. bufsize), advancing *pos.
   Used to build a command's ENTIRE output into one buffer before
   handing it to logger_print_block() as a single atomic block - so a
   concurrent background log line can never land in the middle of a
   "status"/"watchdog"/"events" report. Silently truncates if a
   report ever exceeds bufsize (generous below), rather than
   overflowing. */
static void buf_append(char *buf, size_t bufsize, size_t *pos, const char *fmt, ...)
{
    if (*pos >= bufsize) return;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + *pos, bufsize - *pos, fmt, ap);
    va_end(ap);

    if (n > 0) *pos += (size_t)n;
    if (*pos > bufsize) *pos = bufsize;
}

static void print_help(void)
{
    logger_print_block(
        "Commands:\n"
        "  status    - show current safety state and sensor readings\n"
        "  events    - show the most recent safety events/log entries\n"
        "  watchdog  - show per-task heartbeat/watchdog health\n"
        "  telemetry - show cloud telemetry connection/upload status\n"
        "  estop     - trigger an immediate manual emergency stop\n"
        "  resume    - clear the manual emergency stop\n"
        "  help      - show this message\n"
        "  quit      - stop the motors and exit"
    );
}

static const char *module_display_name(module_id_t m)
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

/* Every data point used below (recovery_manager_get_watchdog_status())
   takes and releases its OWN internal lock (state_lock in
   recovery_manager.c) well before this function ever calls
   logger_print_block() - so logger.c's mutex is never held at the
   same time as any other subsystem's lock. */
static void print_watchdog(void)
{
    char buf[2048];
    size_t pos = 0;

    buf_append(buf, sizeof(buf), &pos, "---- WATCHDOG STATUS ----\n");
    for (int m = 0; m < MODULE_COUNT; m++)
    {
        watchdog_status_t s;
        if (recovery_manager_get_watchdog_status((module_id_t)m, &s) != 0)
        {
            buf_append(buf, sizeof(buf), &pos, "[Watchdog] %-16s NO HEARTBEAT YET\n",
                       module_display_name((module_id_t)m));
            continue;
        }

        const char *label;
        switch (s.state)
        {
            case WATCHDOG_HEALTHY:          label = "HEALTHY"; break;
            case WATCHDOG_MISSED_DEADLINE:  label = "MISSED_DEADLINE"; break;
            case WATCHDOG_PERSISTENT_FAULT: label = "PERSISTENT_FAULT (dead)"; break;
            default:                        label = "?"; break;
        }

        buf_append(buf, sizeof(buf), &pos,
                   "[Watchdog] %-16s %-26s (age=%llums, consecutive misses=%d)\n",
                   module_display_name((module_id_t)m), label,
                   (unsigned long long)s.age_ms, s.consecutive_misses);
    }
    buf_append(buf, sizeof(buf), &pos, "--------------------------");

    logger_print_block(buf);
}

static void print_status(void)
{
    telemetry_t t;
    telemetry_get(&t); /* fast, mutex-protected, released before we return */

    char buf[2048];
    size_t pos = 0;

    buf_append(buf, sizeof(buf), &pos, "---- SAFETY STATUS ----\n");
    buf_append(buf, sizeof(buf), &pos, "State:            %s%s\n", safety_state_label(t.state),
               t.manual_estop_active ? "  (MANUAL ESTOP ACTIVE)" : "");
    buf_append(buf, sizeof(buf), &pos, "Requested speed:  %d%%\n", t.requested_speed_percent);
    buf_append(buf, sizeof(buf), &pos, "Approved speed:   %d%%\n", t.approved_speed_percent);
    buf_append(buf, sizeof(buf), &pos, "Decision latency: %.2fms (deadline bound %dms)\n",
               t.processing_time_ms, t.deadline_bound_ms);
    buf_append(buf, sizeof(buf), &pos, "\n");
    buf_append(buf, sizeof(buf), &pos, "Ultrasonic:  S0=%.1fcm [%s]   S1=%.1fcm [%s]\n",
               t.ultrasonic_cm[0], sensor_health_label(t.ultrasonic_health[0]),
               t.ultrasonic_cm[1], sensor_health_label(t.ultrasonic_health[1]));
    buf_append(buf, sizeof(buf), &pos, "Lane:        %s [%s]\n",
               lane_position_label(t.lane_position), sensor_health_label(t.lane_health));
    buf_append(buf, sizeof(buf), &pos, "Encoder:     L=%.1frpm R=%.1frpm [%s]\n",
               t.left_rpm, t.right_rpm, sensor_health_label(t.encoder_health));
    buf_append(buf, sizeof(buf), &pos, "IMU accel:   x=%.2fg y=%.2fg z=%.2fg [%s]\n",
               t.accel_x, t.accel_y, t.accel_z, sensor_health_label(t.imu_health));
    buf_append(buf, sizeof(buf), &pos, "------------------------");

    logger_print_block(buf);
}

/* Every getter used below (telemetry_get_connection_status(),
   telemetry_get_statistics(), telemetry_service_get_snapshot()) is
   documented as a short, mutex-protected struct copy - same pattern
   as print_watchdog()/print_status() above, so this never holds
   logger.c's mutex at the same time as telemetry_service.c's. */
static void print_telemetry_status(void)
{
    telemetry_conn_status_t     conn;
    telemetry_service_stats_t   stats;
    telemetry_snapshot_t        snap;
    telemetry_get_connection_status(&conn);
    telemetry_get_statistics(&stats);
    telemetry_service_get_snapshot(&snap);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_ms = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);

    const char *state_label;
    switch (conn.state)
    {
        case TELEMETRY_CONN_OFFLINE:    state_label = "OFFLINE"; break;
        case TELEMETRY_CONN_CONNECTING: state_label = "CONNECTING"; break;
        case TELEMETRY_CONN_ONLINE:     state_label = "ONLINE"; break;
        case TELEMETRY_CONN_DEGRADED:   state_label = "DEGRADED"; break;
        case TELEMETRY_CONN_BACKOFF:    state_label = "BACKOFF"; break;
        default:                         state_label = "?"; break;
    }

    char buf[1024];
    size_t pos = 0;

    buf_append(buf, sizeof(buf), &pos, "---- TELEMETRY STATUS ----\n");
    buf_append(buf, sizeof(buf), &pos, "Connection:          %s\n", state_label);
    buf_append(buf, sizeof(buf), &pos, "Robot ID:            %s\n", snap.meta.robot_id);
    buf_append(buf, sizeof(buf), &pos, "Last snapshot seq:   %llu\n",
               (unsigned long long)snap.meta.sequence_number);

    if (conn.last_successful_upload_ms > 0)
        buf_append(buf, sizeof(buf), &pos, "Last successful:     %.1fs ago\n",
                   (double)(now_ms - conn.last_successful_upload_ms) / 1000.0);
    else
        buf_append(buf, sizeof(buf), &pos, "Last successful:     (never)\n");

    if (conn.last_failed_upload_ms > 0)
        buf_append(buf, sizeof(buf), &pos, "Last failed:         %.1fs ago\n",
                   (double)(now_ms - conn.last_failed_upload_ms) / 1000.0);
    else
        buf_append(buf, sizeof(buf), &pos, "Last failed:         (never)\n");

    buf_append(buf, sizeof(buf), &pos, "Consecutive failures:%d\n", conn.consecutive_failures);
    buf_append(buf, sizeof(buf), &pos, "Total successful:    %llu\n",
               (unsigned long long)conn.total_successful_uploads);
    buf_append(buf, sizeof(buf), &pos, "Total failures:      %llu\n",
               (unsigned long long)conn.total_failures);
    buf_append(buf, sizeof(buf), &pos, "\n");
    buf_append(buf, sizeof(buf), &pos, "Snapshot build:      %.2fms\n", (double)stats.snapshot_build_ms);
    buf_append(buf, sizeof(buf), &pos, "Serialization:       %.2fms\n", (double)stats.serialization_ms);
    buf_append(buf, sizeof(buf), &pos, "Network connect:     %.2fms\n", (double)stats.network_connect_ms);
    buf_append(buf, sizeof(buf), &pos, "Network send:        %.2fms\n", (double)stats.network_send_ms);
    buf_append(buf, sizeof(buf), &pos, "Server response:     %.2fms\n", (double)stats.server_response_ms);
    buf_append(buf, sizeof(buf), &pos, "Total upload:        %.2fms\n", (double)stats.total_upload_ms);
    buf_append(buf, sizeof(buf), &pos, "Thread jitter:       %.2fms\n", (double)stats.telemetry_thread_jitter_ms);
    buf_append(buf, sizeof(buf), &pos, "\n");
    buf_append(buf, sizeof(buf), &pos, "Event queue depth:   %d\n", stats.queue_depth);
    buf_append(buf, sizeof(buf), &pos, "Dropped events:      %llu\n",
               (unsigned long long)stats.dropped_events);
    buf_append(buf, sizeof(buf), &pos, "Dropped logs:        %llu\n",
               (unsigned long long)stats.dropped_logs);
    buf_append(buf, sizeof(buf), &pos, "---------------------------");

    logger_print_block(buf);
}

static void print_events(void)
{
    event_entry_t events[16];
    int n = event_log_get_recent(events, 16);

    if (n == 0)
    {
        logger_print_block("(no events logged yet)");
        return;
    }

    char buf[4096];
    size_t pos = 0;

    buf_append(buf, sizeof(buf), &pos, "---- RECENT EVENTS (newest first) ----\n");
    for (int i = 0; i < n; i++)
    {
        buf_append(buf, sizeof(buf), &pos, "[t=%llums] %s\n",
                   (unsigned long long)events[i].time_ms, events[i].message);
    }
    buf_append(buf, sizeof(buf), &pos, "---------------------------------------");

    logger_print_block(buf);
}

/* Puts stdin into raw/non-canonical, no-echo mode so this module can
   do its own character-at-a-time echo through logger_raw_echo() and
   track the in-progress input line through logger_input_set() -
   that's what lets a background log line arriving mid-type erase the
   screen line, print itself, and redraw "> " PLUS whatever the user
   had already typed, instead of tearing into it (the actual failure
   mode this fixes: fgets()'s canonical-mode echo is entirely owned by
   the tty driver, so our process has no way to know what's on screen
   or redraw it). ISIG is left enabled so Ctrl-C/Ctrl-Z still behave
   normally. Returns 0 and fills *orig on success; returns -1 (leaving
   *orig untouched) if stdin isn't a real tty (piped/headless), in
   which case cli_run() falls back to plain fgets() - the pre-existing
   "erase and redraw '> '" behavior is still correct there, since
   there's no in-progress typed text our process can see mid-fgets()
   in canonical mode anyway. */
static int enter_raw_mode(struct termios *orig)
{
    if (!isatty(STDIN_FILENO)) return -1;
    if (tcgetattr(STDIN_FILENO, orig) != 0) return -1;

    struct termios raw = *orig;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return -1;
    return 0;
}

static void leave_raw_mode(const struct termios *orig)
{
    tcsetattr(STDIN_FILENO, TCSANOW, orig);
}

/* Reads one line with our own raw-mode echo, coordinated with
   logger.c (logger_raw_echo() for each echoed byte,
   logger_input_set() after every change) so background log output
   can never tear the line being typed. Returns the line length (0 for
   a blank line), or -1 on EOF/read error. Silently truncates once
   `linesize` is reached, matching fgets()'s prior truncation
   behavior. Unsupported control/escape bytes (arrow keys etc.) are
   ignored - this is a minimal line editor, not a full readline. */
static int read_line_raw(char *line, size_t linesize)
{
    size_t len = 0;
    line[0] = '\0';

    for (;;)
    {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) return -1;

        if (c == '\n' || c == '\r')
        {
            logger_raw_echo("\n", 1);
            return (int)len;
        }
        else if (c == 127 || c == 8) /* DEL or backspace */
        {
            if (len > 0)
            {
                len--;
                line[len] = '\0';
                logger_raw_echo("\b \b", 3); /* erase the char on screen */
                logger_input_set(line);
            }
        }
        else if ((unsigned char)c < 0x20)
        {
            continue; /* other control bytes - ignored, not lost data */
        }
        else if (len + 1 < linesize)
        {
            line[len++] = c;
            line[len]   = '\0';
            logger_raw_echo(&c, 1);
            logger_input_set(line);
        }
        /* else: line already at capacity - drop further characters,
           same as fgets(line, sizeof(line), stdin) truncating. */
    }
}

void cli_run(void)
{
    char line[128];
    struct termios orig_termios;
    int raw_ok = (enter_raw_mode(&orig_termios) == 0);

    logger_print_block("\nAutonomous Vehicle Safety Supervisor - CLI ready.\n"
                        "Type 'help' for commands.");

    for (;;)
    {
        /* Marks the prompt visible so a background log line arriving
           while we're blocked reading below moves to its own line and
           redraws "> " (+ in-progress input, when raw_ok) afterwards,
           instead of tearing into it. */
        logger_show_prompt("> ");

        int got;
        if (raw_ok)
        {
            got = read_line_raw(line, sizeof(line));
        }
        else
        {
            got = (fgets(line, sizeof(line), stdin) != NULL) ? (int)strlen(line) : -1;
        }

        if (got < 0)
        {
            /* stdin closed (e.g. running headless/detached) - keep the
               process alive rather than exiting, since the rest of
               the system should keep running without a CLI attached. */
            logger_clear_prompt_visible();
            logger_print_block("(stdin closed - CLI disabled, system keeps running)");
            if (raw_ok) leave_raw_mode(&orig_termios);
            for (;;) pause();
        }

        /* The prompt line has now been submitted - background output
           printed from here until we draw the next prompt is just
           ordinary sequential output, not a redraw case. */
        logger_clear_prompt_visible();

        /* fgets() leaves a trailing newline; read_line_raw() never
           adds one to `line` itself (it only echoes "\n" to the
           screen), so this is a no-op but harmless in the raw path. */
        line[strcspn(line, "\n")] = '\0';

        if (strcmp(line, "help") == 0)
        {
            print_help();
        }
        else if (strcmp(line, "status") == 0)
        {
            print_status();
        }
        else if (strcmp(line, "events") == 0)
        {
            print_events();
        }
        else if (strcmp(line, "watchdog") == 0)
        {
            print_watchdog();
        }
        else if (strcmp(line, "telemetry") == 0 || strcmp(line, "telemetry status") == 0)
        {
            print_telemetry_status();
        }
        else if (strcmp(line, "estop") == 0)
        {
            atomic_store(&estop_active, 1);
            motor_controller_emergency_stop_now();
            event_log_add("MANUAL ESTOP triggered via CLI");
            logger_print_block("Manual emergency stop ACTIVE. Motors stopped. "
                                "Type 'resume' to clear.");
        }
        else if (strcmp(line, "resume") == 0)
        {
            atomic_store(&estop_active, 0);
            event_log_add("Manual ESTOP cleared via CLI");
            logger_print_block("Manual emergency stop cleared - normal operation "
                                "resumes on the next tick.");
        }
        else if (strcmp(line, "quit") == 0 || strcmp(line, "exit") == 0)
        {
            motor_controller_emergency_stop_now();
            logger_print_block("Motors stopped. Exiting.");
            if (raw_ok) leave_raw_mode(&orig_termios); /* leave the terminal usable */
            _exit(0);
        }
        else if (strlen(line) == 0)
        {
            /* blank line - ignore */
        }
        else
        {
            logger_log("Unknown command '%s'. Type 'help' for commands.", line);
        }
    }
}
