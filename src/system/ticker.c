/*
 * ticker.c
 * ---------------------------------------------------------------
 * Periodic loops on QNX are NOT done with sleep()/usleep() (they
 * drift: the work time gets added to the sleep time). Instead we ask
 * the kernel for a periodic TIMER that sends us a PULSE every period.
 * The task then just does MsgReceive() and wakes up exactly on time.
 */
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../system/ticker.h"

/* timer_pulse_start:
 *   1. ConnectAttach to our own channel so the kernel has a
 *      connection id (coid) to deliver the pulse through.
 *   2. Build a sigevent that says "deliver a PULSE with this code".
 *      SIGEV_PULSE_PRIO_INHERIT = the pulse gets the priority of the
 *      thread that created the timer (so a high-priority task is
 *      woken at high priority).
 *   3. timer_create + timer_settime: first expiry after one period,
 *      then repeat every period.                                    */
int timer_pulse_start(int chid, int pulse_code, int period_ms)
{
    struct sigevent ev;
    struct itimerspec its;
    timer_t timer_id;
    int coid;

    coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        perror("ticker: ConnectAttach");
        return -1;
    }

    SIGEV_PULSE_INIT(&ev, coid, SIGEV_PULSE_PRIO_INHERIT, pulse_code, 0);

    if (timer_create(CLOCK_MONOTONIC, &ev, &timer_id) == -1) {
        perror("ticker: timer_create");
        return -1;
    }

    its.it_value.tv_sec     = period_ms / 1000;
    its.it_value.tv_nsec    = (long)(period_ms % 1000) * 1000000L;
    its.it_interval         = its.it_value;      /* repeat forever */

    if (timer_settime(timer_id, 0, &its, NULL) == -1) {
        perror("ticker: timer_settime");
        return -1;
    }
    return 0;
}
