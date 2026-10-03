/*
 * messages.h
 * ---------------------------------------------------------------
 * Every message / pulse that travels between our tasks is defined
 * here. This is the "protocol" of the system.
 *
 * QNX message passing in one paragraph:
 *   - A server creates a CHANNEL (ChannelCreate) and sits in
 *     MsgReceive().
 *   - A client CONNECTS to that channel (ConnectAttach -> "coid")
 *     and calls MsgSend(). MsgSend BLOCKS until the server calls
 *     MsgReply(). This synchronous hand-shake is what makes QNX
 *     IPC deterministic.
 *   - A PULSE is a tiny non-blocking message (MsgSendPulse) - we use
 *     it for heartbeats and for timer ticks.
 */
#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <sys/neutrino.h>
#include "../common/config.h"

/* ---------- Enums shared by several tasks -------------------------- */

/* What Navigation WANTS to do. */
typedef enum {
    CMD_STOP = 0,
    CMD_FORWARD,
    CMD_TURN_RIGHT
} nav_cmd_t;

/* What Safety DECIDED. Printed in the logs too. */
typedef enum {
    STATE_SAFE = 0,     /* request approved as is                   */
    STATE_WARNING,      /* obstacle getting close, speed reduced    */
    STATE_EMERGENCY,    /* obstacle too close, hard stop            */
    STATE_FAULT         /* sensors can't be trusted, hard stop      */
} safety_state_t;

/* Health of ONE ultrasonic sensor as judged by the Sensor Monitor
 * (see sensors/sensor_health.c). Safety only trusts SENSOR_OK.      */
typedef enum {
    SENSOR_WAITING = 0, /* not enough good readings yet (start-up)   */
    SENSOR_OK,          /* filtered distance can be trusted          */
    SENSOR_FAULT        /* too many bad readings, NOT trusted        */
} sensor_status_t;

/* Task ids - used as the "value" inside a heartbeat pulse. */
typedef enum {
    TASK_SENSOR = 0,
    TASK_NAVIGATION,
    TASK_SAFETY,
    TASK_MOTOR,
    NUM_TASKS
} task_id_t;

/* ---------- Pulse codes -------------------------------------------- */
/* User pulse codes must start at _PULSE_CODE_MINAVAIL.              */
#define PULSE_TICK        (_PULSE_CODE_MINAVAIL + 0)  /* periodic timer */
#define PULSE_HEARTBEAT   (_PULSE_CODE_MINAVAIL + 1)  /* "I'm alive"    */
#define PULSE_WD_CHECK    (_PULSE_CODE_MINAVAIL + 2)  /* watchdog timer */
#define PULSE_DISPLAY     (_PULSE_CODE_MINAVAIL + 3)  /* Safety -> OLED */

/* ---------- Message types ------------------------------------------ */
/* First field of every message so the receiver knows what it got.   */
#define MSG_NAV_REQUEST   1
#define MSG_SENSOR_DATA   2
#define MSG_MOTOR_CMD     3

/* Navigation -> Safety */
typedef struct {
    uint16_t  type;             /* MSG_NAV_REQUEST                    */
    nav_cmd_t cmd;
    int       speed_percent;    /* 0..100                             */
} nav_msg_t;

/* Safety -> Navigation (reply to nav_msg_t) */
typedef struct {
    safety_state_t state;
    int            approved_speed;   /* what Safety really allowed    */
} nav_reply_t;

/* Sensor Monitor -> Safety. The distances are already MEDIAN-FILTERED
 * and each sensor carries its own status, so Safety never sees raw
 * noisy readings. distance_cm[i] is only meaningful when
 * status[i] == SENSOR_OK.                                            */
typedef struct {
    uint16_t        type;                      /* MSG_SENSOR_DATA     */
    float           distance_cm[NUM_SENSORS];  /* filtered, in cm     */
    sensor_status_t status[NUM_SENSORS];       /* WAITING / OK / FAULT */
} sensor_msg_t;

/* Safety (or Watchdog) -> Motor Controller */
typedef struct {
    uint16_t  type;             /* MSG_MOTOR_CMD                      */
    nav_cmd_t cmd;
    int       speed_percent;
} motor_msg_t;

/* Generic "OK" reply used for sensor and motor messages. */
typedef struct {
    int ok;
} ack_reply_t;

/* The buffer Safety receives into. It has to be big enough for the
 * biggest message it can get, plus a struct _pulse in case a pulse
 * arrives (MsgReceive returns 0 for those). A union does that. */
typedef union {
    uint16_t      type;
    nav_msg_t     nav;
    sensor_msg_t  sensor;
    struct _pulse pulse;
} safety_in_t;

#endif /* MESSAGES_H */
