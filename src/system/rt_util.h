/*
 * rt_util.h - small real-time helpers used by every task (see rt_util.c)
 */
#ifndef RT_UTIL_H
#define RT_UTIL_H

#include <stdint.h>
#include <pthread.h>

/* Milliseconds from CLOCK_MONOTONIC (never jumps backwards). */
uint64_t now_ms(void);

/* Microseconds from CLOCK_MONOTONIC - for measuring short durations. */
uint64_t now_us(void);

/* Start a thread with SCHED_FIFO at the given priority. */
int create_rt_thread(void *(*fn)(void *), void *arg, int priority, const char *name);

/* MsgSend() that gives up after timeout_ms instead of blocking forever.
 * Returns what MsgSend returns (-1 and errno=ETIMEDOUT on timeout). */
int msg_send_timeout(int coid, const void *smsg, int sbytes,
                     void *rmsg, int rbytes, int timeout_ms);

#endif
