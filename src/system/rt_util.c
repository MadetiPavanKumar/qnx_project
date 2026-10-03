/*
 * rt_util.c
 * ---------------------------------------------------------------
 * Helpers that every task needs: reading the clock, creating a
 * real-time thread, and sending a message with a timeout.
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include "../system/rt_util.h"

/* now_us: current CLOCK_MONOTONIC time in microseconds.
 * MONOTONIC is used because wall-clock time can be changed by NTP or
 * the user, which would break our deadline measurements. */
uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* now_ms: same clock, milliseconds. */
uint64_t now_ms(void)
{
    return now_us() / 1000ULL;
}

/* create_rt_thread:
 * Creates a detached thread using the SCHED_FIFO policy.
 *   - SCHED_FIFO = fixed priority, no time slicing: a thread runs
 *     until it blocks or a higher-priority thread becomes ready.
 *   - PTHREAD_EXPLICIT_SCHED is REQUIRED, otherwise the new thread
 *     silently inherits the priority of the creator (main) and our
 *     priority numbers would be ignored.
 * If the process isn't allowed to use real-time priorities (not root /
 * no PROCMGR_AID_SCHEDULE ability) we print a warning and fall back to
 * a normal thread so the demo still runs. */
int create_rt_thread(void *(*fn)(void *), void *arg, int priority, const char *name)
{
    pthread_attr_t attr;
    struct sched_param param;
    pthread_t tid;
    int rc;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    memset(&param, 0, sizeof(param));
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    rc = pthread_create(&tid, &attr, fn, arg);
    pthread_attr_destroy(&attr);

    if (rc != 0) {
        fprintf(stderr, "warning: %s: SCHED_FIFO prio %d failed (%s) - "
                "using default scheduling\n", name, priority, strerror(rc));

        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        rc = pthread_create(&tid, &attr, fn, arg);
        pthread_attr_destroy(&attr);
        if (rc != 0) {
            fprintf(stderr, "error: %s: pthread_create failed (%s)\n", name, strerror(rc));
            return -1;
        }
    }

    pthread_setname_np(tid, name);   /* shows up in 'pidin' / debugger */
    return 0;
}

/* msg_send_timeout:
 * A plain MsgSend() blocks until the server replies - if the server is
 * stuck, the caller is stuck too. In a safety system we never want to
 * wait forever. QNX lets us arm a kernel timeout with TimerTimeout()
 * right BEFORE a blocking call: if the thread is still in the
 * SEND-blocked or REPLY-blocked state when the time is up, the kernel
 * unblocks it and MsgSend returns -1 with errno = ETIMEDOUT. */
int msg_send_timeout(int coid, const void *smsg, int sbytes,
                     void *rmsg, int rbytes, int timeout_ms)
{
    struct sigevent ev;
    uint64_t timeout_ns = (uint64_t)timeout_ms * 1000000ULL;

    SIGEV_UNBLOCK_INIT(&ev);
    TimerTimeout(CLOCK_MONOTONIC,
                 _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                 &ev, &timeout_ns, NULL);

    return MsgSend(coid, smsg, sbytes, rmsg, rbytes);
}
