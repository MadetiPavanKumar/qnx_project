/*
 * motor_controller.c
 * ---------------------------------------------------------------
 * The Motor Controller is a dumb executor: it does NOT decide
 * anything. Safety has already decided what is allowed, we just
 * obey. That separation is the whole idea of the project -
 * Navigation proposes, Safety disposes, Motor Controller executes.
 *
 * Extra protection: a DEAD-MAN TIMER. If no command arrives for
 * MOTOR_TIMEOUT_MS (e.g. Safety crashed) the motors are stopped by
 * this task itself. It is implemented with a kernel receive-timeout
 * (TimerTimeout), no extra thread needed.
 */
#include <stdio.h>
#include <errno.h>
#include <sys/neutrino.h>
#include "../motor/motor_controller.h"
#include "../system/watchdog.h"
#include "../drivers/drv8833.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"

static int motor_chid = -1;

/* motor_stop: stop both wheels (PWM duty 0, all direction pins low). */
static void motor_stop(void)
{
    drv8833_stop();
}

/* motor_drive: translate our command into a DRV8833 driver call.
 * The board has ONE shared speed for both wheels, so we can only go
 * straight or pivot - good enough for this project. */
static void motor_drive(nav_cmd_t cmd, int speed_percent)
{
    if (cmd == CMD_FORWARD)
        drv8833_forward((unsigned)speed_percent);
    else if (cmd == CMD_TURN_RIGHT)
        drv8833_right((unsigned)speed_percent);
    else
        drv8833_stop();
}

/* motor_controller_get_chid: other modules use this to attach. */
int motor_controller_get_chid(void)
{
    return motor_chid;
}

/* cmd_name: command -> text for the logs. */
static const char *cmd_name(nav_cmd_t cmd)
{
    switch (cmd) {
    case CMD_FORWARD:    return "FORWARD";
    case CMD_TURN_RIGHT: return "TURN_RIGHT";
    default:             return "STOP";
    }
}

/* motor_thread: main loop.
 *   1. Arm a 'receive timeout' with TimerTimeout().
 *   2. Block in MsgReceive():
 *        - got a message  -> drive the motors, MsgReply an ack
 *        - timed out      -> nobody is talking to us -> STOP
 *   3. Send a heartbeat to the watchdog either way (the timeout also
 *      wakes us regularly, so a silent system still produces
 *      heartbeats while we are alive).
 * The motors are only touched when the command CHANGES, which avoids
 * hammering the GPIO driver with identical writes every 75 ms. */
static void *motor_thread(void *arg)
{
    union { uint16_t type; motor_msg_t cmd; struct _pulse pulse; } in;
    ack_reply_t ack = { 1 };
    struct sigevent ev;
    uint64_t timeout_ns = (uint64_t)MOTOR_TIMEOUT_MS * 1000000ULL;
    nav_cmd_t last_cmd   = CMD_STOP;
    int       last_speed = 0;
    int       dead_man_active = 0;
    int       wd = watchdog_connect();
    (void)arg;

    motor_stop();    /* start in a known safe state */

    for (;;) {
        SIGEV_UNBLOCK_INIT(&ev);
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, &ev, &timeout_ns, NULL);

        int rcvid = MsgReceive(motor_chid, &in, sizeof(in), NULL);
        watchdog_beat(wd, TASK_MOTOR);

        if (rcvid == -1) {
            if (errno == ETIMEDOUT && !dead_man_active) {
                /* Dead-man timer fired: log once, stop, keep waiting. */
                dead_man_active = 1;
                motor_stop();
                last_cmd = CMD_STOP;
                last_speed = 0;
                log_msg("[MOTOR] no command for %d ms -> dead-man STOP", MOTOR_TIMEOUT_MS);
            }
            continue;
        }
        if (rcvid == 0)
            continue;               /* a pulse - nothing to do */

        if (in.type != MSG_MOTOR_CMD) {
            MsgError(rcvid, ENOSYS);    /* unknown message type */
            continue;
        }

        dead_man_active = 0;        /* we heard from someone again */

        if (in.cmd.cmd != last_cmd || in.cmd.speed_percent != last_speed) {
            if (in.cmd.cmd == CMD_STOP || in.cmd.speed_percent == 0)
                motor_stop();
            else
                motor_drive(in.cmd.cmd, in.cmd.speed_percent);

            last_cmd   = in.cmd.cmd;
            last_speed = in.cmd.speed_percent;
            log_msg("[MOTOR] %s %d%%", cmd_name(last_cmd), last_speed);
        }

        MsgReply(rcvid, EOK, &ack, sizeof(ack));
    }
    return NULL;
}

/* motor_controller_start:
 *   1. configure the DRV8833 driver (pins from config.h); refuse to
 *      start if that fails
 *   2. channel first, then thread (so that clients started afterwards
 *      can already attach to it). */
int motor_controller_start(void)
{
    drv8833_config_t cfg;

    cfg.en_pin        = PIN_MOTOR_EN;
    cfg.right_fwd_pin = PIN_MOTOR_RIGHT_FWD;
    cfg.right_rev_pin = PIN_MOTOR_RIGHT_REV;
    cfg.left_fwd_pin  = PIN_MOTOR_LEFT_FWD;
    cfg.left_rev_pin  = PIN_MOTOR_LEFT_REV;
    cfg.pwm_frequency = MOTOR_PWM_HZ;
    if (drv8833_init(&cfg) != DRV8833_OK) {
        fprintf(stderr, "motor_controller: drv8833_init failed\n");
        return -1;
    }

    motor_chid = ChannelCreate(0);
    if (motor_chid == -1) {
        perror("motor: ChannelCreate");
        return -1;
    }
    return create_rt_thread(motor_thread, NULL, PRIO_MOTOR, "motor_ctrl");
}

/* motor_controller_close: shutdown only. emergency_stop does not wait
 * on the driver mutex, so it cannot be blocked by another thread. */
void motor_controller_close(void)
{
    drv8833_emergency_stop();
    drv8833_deinit();
}
