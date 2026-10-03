/*
 * watchdog.c
 * ---------------------------------------------------------------
 * Our simple version of a "Recovery Manager":
 *
 *   - Every task sends a HEARTBEAT PULSE here regularly.
 *   - A periodic timer pulse makes the watchdog check how long ago
 *     each task last sent one.
 *   - If a task is silent for too long it is declared MISSING: we log
 *     it and force the motors to stop (fail-safe).
 *   - When heartbeats start again we log RECOVERED.
 *
 * The watchdog only ever receives PULSES, so tasks that send it a
 * heartbeat never block, even if the watchdog is busy.
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../system/watchdog.h"
#include "../motor/motor_controller.h"
#include "../system/ticker.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"

static int wd_chid = -1;                    /* our channel               */

static const char *task_name[NUM_TASKS] = {
    "SensorMonitor", "Navigation", "Safety", "MotorController"
};

/* watchdog_get_chid: lets other modules find our channel. */
int watchdog_get_chid(void)
{
    return wd_chid;
}

/* watchdog_connect: every task calls this once to get a connection
 * (coid) to the watchdog's channel. */
int watchdog_connect(void)
{
    return ConnectAttach(ND_LOCAL_NODE, 0, wd_chid, _NTO_SIDE_CHANNEL, 0);
}

/* watchdog_beat: MsgSendPulse is NON-blocking - it drops a small
 * pulse into the watchdog's queue and returns immediately.
 * Priority -1 means "deliver at my own priority". */
void watchdog_beat(int wd_coid, task_id_t who)
{
    MsgSendPulse(wd_coid, -1, PULSE_HEARTBEAT, (int)who);
}

/* force_motor_stop: ask the Motor Controller to stop, using a message
 * with a timeout so a hung Motor Controller can't hang US as well. */
static void force_motor_stop(int motor_coid)
{
    motor_msg_t msg;
    ack_reply_t reply;

    msg.type          = MSG_MOTOR_CMD;
    msg.cmd           = CMD_STOP;
    msg.speed_percent = 0;
    msg_send_timeout(motor_coid, &msg, sizeof(msg), &reply, sizeof(reply), 50);
}

/* timeout_for: heartbeat deadline for one task. The Sensor Monitor gets
 * a longer one because reading ultrasonic sensors is slower. */
static int timeout_for(task_id_t id)
{
    return (id == TASK_SENSOR) ? SENSOR_HEARTBEAT_TIMEOUT_MS
                               : HEARTBEAT_TIMEOUT_MS;
}

/* watchdog_thread: main loop.
 *   pulse HEARTBEAT -> remember the time; if the task had been marked
 *                      missing, log that it recovered.
 *   pulse WD_CHECK  -> look at all tasks, flag the ones that are late. */
static void *watchdog_thread(void *arg)
{
    struct _pulse pulse;
    uint64_t last_beat[NUM_TASKS];
    int alive[NUM_TASKS];
    int motor_coid = -1;
    int i;
    (void)arg;

    /* Start the periodic check timer (pulse -> our own channel). */
    if (timer_pulse_start(wd_chid, PULSE_WD_CHECK, WATCHDOG_CHECK_MS) != 0)
        return NULL;

    /* Give every task a fresh start so we don't flag them at boot. */
    for (i = 0; i < NUM_TASKS; i++) {
        last_beat[i] = now_ms();
        alive[i]     = 1;
    }

    for (;;) {
        int rcvid = MsgReceive(wd_chid, &pulse, sizeof(pulse), NULL);
        if (rcvid != 0)         /* we only expect pulses (rcvid == 0) */
            continue;

        if (pulse.code == PULSE_HEARTBEAT) {
            int id = pulse.value.sival_int;
            if (id < 0 || id >= NUM_TASKS)
                continue;
            last_beat[id] = now_ms();
            if (!alive[id]) {
                alive[id] = 1;
                log_msg("[WATCHDOG] %s RECOVERED (heartbeat is back)", task_name[id]);
            }
        }
        else if (pulse.code == PULSE_WD_CHECK) {
            uint64_t now = now_ms();

            /* Lazy connect: the Motor Controller is started after us. */
            if (motor_coid < 0 && motor_controller_get_chid() >= 0)
                motor_coid = ConnectAttach(ND_LOCAL_NODE, 0, motor_controller_get_chid(),
                                           _NTO_SIDE_CHANNEL, 0);

            for (i = 0; i < NUM_TASKS; i++) {
                if (alive[i] && (now - last_beat[i]) > (uint64_t)timeout_for((task_id_t)i)) {
                    alive[i] = 0;        /* log only ONCE per failure */
                    log_msg("[WATCHDOG] %s MISSED heartbeat (%llu ms) -> forcing motors STOP",
                            task_name[i], (unsigned long long)(now - last_beat[i]));
                    if (motor_coid >= 0)
                        force_motor_stop(motor_coid);
                }
            }
        }
    }
    return NULL;
}

/* watchdog_start: create the channel FIRST (so other tasks can attach
 * to it as soon as this function returns), then start the thread. */
int watchdog_start(void)
{
    wd_chid = ChannelCreate(0);
    if (wd_chid == -1) {
        perror("watchdog: ChannelCreate");
        return -1;
    }
    return create_rt_thread(watchdog_thread, NULL, PRIO_WATCHDOG, "watchdog");
}
