/*
 * navigation.c
 * ---------------------------------------------------------------
 * Navigation is intentionally simple (and a bit naive): it just wants
 * to drive forward at cruise speed. It has NO idea what the sensors
 * say. Every tick it sends its wish to the Safety Supervisor and
 * learns - from the reply - what was actually allowed.
 *
 * If Safety keeps saying EMERGENCY for a while, Navigation gives up
 * on going straight and pivots right for a moment, then tries
 * forward again. Even then, every command still goes through Safety.
 */
#include <stdio.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../navigation/navigation.h"
#include "../safety/safety_supervisor.h"
#include "../system/watchdog.h"
#include "../system/ticker.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"

/* navigation_thread: runs exactly once per TICK_MS.
 *   1. Block on the timer pulse (that's our fixed-rate loop).
 *   2. Decide what we WANT: forward, or keep turning if mid-turn.
 *   3. MsgSend the request to Safety. We stay blocked until Safety
 *      replies with the approved speed.
 *   4. Use the reply to detect a blocked path.
 *   5. Heartbeat to the watchdog.                                    */
static void *navigation_thread(void *arg)
{
    struct _pulse pulse;
    nav_msg_t req;
    nav_reply_t reply;
    int tick_chid, safety_coid, wd;
    int blocked_ticks = 0;     /* consecutive EMERGENCY replies        */
    int turn_ticks_left = 0;   /* >0 while pivoting                    */
    int send_failed_logged = 0;
    (void)arg;

    tick_chid = ChannelCreate(0);
    if (tick_chid == -1 || timer_pulse_start(tick_chid, PULSE_TICK, TICK_MS) != 0)
        return NULL;

    safety_coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_get_chid(), _NTO_SIDE_CHANNEL, 0);
    wd = watchdog_connect();
    if (safety_coid == -1) {
        perror("navigation: ConnectAttach(safety)");
        return NULL;
    }

    for (;;) {
        MsgReceive(tick_chid, &pulse, sizeof(pulse), NULL);   /* wait for tick */

        req.type = MSG_NAV_REQUEST;
        if (turn_ticks_left > 0) {
            req.cmd           = CMD_TURN_RIGHT;
            req.speed_percent = NAV_TURN_SPEED;
            turn_ticks_left--;
            if (turn_ticks_left == 0)
                log_msg("[NAV] turn finished -> trying forward again");
        } else {
            req.cmd           = CMD_FORWARD;
            req.speed_percent = NAV_CRUISE_SPEED;
        }

        if (msg_send_timeout(safety_coid, &req, sizeof(req),
                             &reply, sizeof(reply), TICK_MS * 2) == -1) {
            if (!send_failed_logged) {
                log_msg("[NAV] Safety did not answer (%s)", "timeout/error");
                send_failed_logged = 1;
            }
        } else {
            send_failed_logged = 0;

            /* Count how long Safety has been refusing to let us go. */
            if (req.cmd == CMD_FORWARD) {
                if (reply.state == STATE_EMERGENCY)
                    blocked_ticks++;
                else
                    blocked_ticks = 0;

                if (blocked_ticks >= NAV_BLOCKED_TICKS) {
                    log_msg("[NAV] path blocked -> pivoting right");
                    turn_ticks_left = NAV_TURN_TICKS;
                    blocked_ticks = 0;
                }
            }
        }

        watchdog_beat(wd, TASK_NAVIGATION);
    }
    return NULL;
}

/* navigation_start: just launches the thread (it attaches to Safety's
 * channel itself, which already exists because main starts Safety
 * first). */
int navigation_start(void)
{
    return create_rt_thread(navigation_thread, NULL, PRIO_NAVIGATION, "navigation");
}
