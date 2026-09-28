#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

#include "config.h"

/* Generous enough for every existing log line/stats line in the
   project; vsnprintf() truncates safely if a caller ever exceeds
   this rather than overflowing. */
#define LOGGER_LINE_MAX  256

/* ---- Terminal state (Logger task + CLI thread only - see the
   design note in logger.h; no real-time task ever touches this) ---- */
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static atomic_int       prompt_visible = 0;

#define LOGGER_PROMPT_MAX 32
static char prompt_text[LOGGER_PROMPT_MAX]  = "> ";
static char input_snapshot[LOGGER_LINE_MAX] = "";

/* ---- Bounded queue (any thread may push; only the Logger task
   pops) ---- */
typedef struct
{
    log_level_t level;
    int         is_stderr;
    char        text[LOGGER_LINE_MAX];
} log_record_t;

static log_record_t    queue_buf[LOGGER_QUEUE_CAPACITY];
static int              q_head  = 0; /* next slot to pop */
static int              q_tail  = 0; /* next free slot to push */
static int              q_count = 0;
static pthread_mutex_t  queue_lock       = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   queue_not_empty  = PTHREAD_COND_INITIALIZER;

static atomic_int             current_level = LOG_LEVEL_INFO;
static atomic_uint_least64_t  dropped_logs  = 0;

/* Core primitive: print one already-formatted, complete block to
   `stream`, atomically with respect to the CLI prompt and every
   other caller of this function (the Logger task and the CLI
   thread - see logger.h). This is the ONLY place that takes
   log_mutex, and the ONLY work done while holding it is the write
   itself (plus, when the prompt was visible, the one necessary
   erase-and-redraw) - no formatting, no other subsystem's lock, no
   sensor/IPC/hardware call ever happens in this critical section. */
static void print_block_stream(FILE *stream, const char *text)
{
    pthread_mutex_lock(&log_mutex);

    int was_prompt_visible = atomic_exchange(&prompt_visible, 0);
    if (was_prompt_visible)
    {
        fputc('\n', stdout); /* move off the "> " line before logging */
    }

    fputs(text, stream);
    size_t len = strlen(text);
    if (len == 0 || text[len - 1] != '\n')
    {
        fputc('\n', stream);
    }
    fflush(stream);

    if (was_prompt_visible)
    {
        fputs(prompt_text, stdout);
        fputs(input_snapshot, stdout);
        fflush(stdout);
        atomic_store(&prompt_visible, 1);
    }

    pthread_mutex_unlock(&log_mutex);
}

/* Bounded, O(1), safe to call from a real-time thread - see the
   overflow policy in logger.h's design note. */
static void enqueue(log_level_t level, int is_stderr, const char *text)
{
    pthread_mutex_lock(&queue_lock);

    if (q_count == LOGGER_QUEUE_CAPACITY)
    {
        if (queue_buf[q_head].level == LOG_LEVEL_DEBUG)
        {
            /* Discard the oldest (DEBUG) entry to make room. */
            q_head = (q_head + 1) % LOGGER_QUEUE_CAPACITY;
            q_count--;
            atomic_fetch_add(&dropped_logs, 1);
        }
        else
        {
            /* Queue is full of INFO/WARN/ERROR - drop the incoming
               entry rather than something already waiting. */
            atomic_fetch_add(&dropped_logs, 1);
            pthread_mutex_unlock(&queue_lock);
            return;
        }
    }

    log_record_t *slot = &queue_buf[q_tail];
    slot->level     = level;
    slot->is_stderr = is_stderr;
    snprintf(slot->text, sizeof(slot->text), "%s", text);

    q_tail = (q_tail + 1) % LOGGER_QUEUE_CAPACITY;
    q_count++;

    pthread_cond_signal(&queue_not_empty);
    pthread_mutex_unlock(&queue_lock);
}

static void log_formatted(log_level_t level, int is_stderr, const char *fmt, va_list ap)
{
    if ((int)level < atomic_load(&current_level)) return; /* filtered, not dropped */

    char line[LOGGER_LINE_MAX];
    vsnprintf(line, sizeof(line), fmt, ap); /* formatting done OUTSIDE any lock */
    enqueue(level, is_stderr, line);
}

void logger_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_formatted(LOG_LEVEL_INFO, 0, fmt, ap);
    va_end(ap);
}

void logger_log_err(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_formatted(LOG_LEVEL_ERROR, 1, fmt, ap);
    va_end(ap);
}

void logger_debug(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_formatted(LOG_LEVEL_DEBUG, 0, fmt, ap);
    va_end(ap);
}

void logger_warn(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_formatted(LOG_LEVEL_WARN, 0, fmt, ap);
    va_end(ap);
}

void logger_print_block(const char *text)
{
    /* Called only from the CLI thread (its own command output) - a
       direct synchronous print, not queued, since the CLI thread
       blocking briefly on its own terminal is not a real-time
       concern. See logger.h. */
    print_block_stream(stdout, text);
}

void logger_show_prompt(const char *prompt)
{
    pthread_mutex_lock(&log_mutex);
    snprintf(prompt_text, sizeof(prompt_text), "%s", prompt);
    input_snapshot[0] = '\0';
    fputs(prompt_text, stdout);
    fflush(stdout);
    atomic_store(&prompt_visible, 1);
    pthread_mutex_unlock(&log_mutex);
}

void logger_input_set(const char *current_line)
{
    pthread_mutex_lock(&log_mutex);
    snprintf(input_snapshot, sizeof(input_snapshot), "%s", current_line);
    pthread_mutex_unlock(&log_mutex);
}

void logger_raw_echo(const char *bytes, size_t len)
{
    if (len == 0) return;

    pthread_mutex_lock(&log_mutex);
    fwrite(bytes, 1, len, stdout);
    fflush(stdout);
    pthread_mutex_unlock(&log_mutex);
}

void logger_clear_prompt_visible(void)
{
    atomic_store(&prompt_visible, 0);
}

void logger_set_level(log_level_t level)
{
    atomic_store(&current_level, level);
}

log_level_t logger_get_level(void)
{
    return (log_level_t)atomic_load(&current_level);
}

uint64_t logger_get_dropped_count(void)
{
    return (uint64_t)atomic_load(&dropped_logs);
}

/* The Logger task itself: blocks on the queue, prints whatever it
   pops, one record at a time, on its OWN low-priority thread. This
   is the only thread (besides the CLI thread) that ever calls
   print_block_stream() / touches log_mutex. */
static void *logger_thread_fn(void *arg)
{
    (void)arg;

    for (;;)
    {
        pthread_mutex_lock(&queue_lock);
        while (q_count == 0)
        {
            pthread_cond_wait(&queue_not_empty, &queue_lock);
        }

        log_record_t rec = queue_buf[q_head];
        q_head = (q_head + 1) % LOGGER_QUEUE_CAPACITY;
        q_count--;

        pthread_mutex_unlock(&queue_lock);

        print_block_stream(rec.is_stderr ? stderr : stdout, rec.text);
    }

    return NULL; /* unreachable */
}

void logger_start(void)
{
    pthread_t tid;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = PRIORITY_LOGGER;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&tid, &attr, logger_thread_fn, NULL) != 0)
    {
        /* Logger itself couldn't start - fall back to printing this
           one message directly, since logger_log() would just queue
           into a buffer nothing will ever drain. */
        fprintf(stderr, "logger_start: pthread_create failed - logging "
                        "will not be available\n");
        return;
    }
    pthread_detach(tid);
}
