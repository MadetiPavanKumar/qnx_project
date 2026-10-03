/*
 * log.c
 * ---------------------------------------------------------------
 * A very small logger. Several threads print at the same time, so
 * we protect printf with a mutex to stop lines getting mixed up.
 *
 * QNX concept used here: PRIORITY INHERITANCE on the mutex.
 * Imagine the low-priority Sensor thread holds the log mutex and
 * gets pre-empted by a medium-priority thread; then the high-
 * priority Safety thread wants to log and has to wait for the
 * low-priority one -> priority inversion. With PTHREAD_PRIO_INHERIT
 * the mutex owner is temporarily boosted to the priority of the
 * highest waiter, so the inversion is bounded.
 *
 * (We still only log on EVENTS, not on every tick, because console
 * I/O is slow and not deterministic.)
 */
#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include "../common/log.h"
#include "../system/rt_util.h"

static pthread_mutex_t log_lock;      /* protects stdout               */
static uint64_t        start_ms;      /* time of log_init()            */

/* log_init: sets up the priority-inheriting mutex and remembers the
 * start time so timestamps are relative to program start. */
void log_init(void)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&log_lock, &attr);
    pthread_mutexattr_destroy(&attr);

    start_ms = now_ms();
}

/* log_msg: formats the message into a local buffer first (outside
 * the lock, to keep the locked section short) and then prints it
 * with a timestamp while holding the mutex. */
void log_msg(const char *fmt, ...)
{
    char buf[256];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&log_lock);
    printf("[%7llu ms] %s\n", (unsigned long long)(now_ms() - start_ms), buf);
    fflush(stdout);
    pthread_mutex_unlock(&log_lock);
}
