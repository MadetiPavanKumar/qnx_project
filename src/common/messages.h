#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include "system_types.h"

typedef enum
{
    MSG_NAVIGATION,
    MSG_SENSOR_DATA,
    MSG_SYSTEM,
    MSG_HEARTBEAT
} message_type_t;

/* What Navigation wants to do THIS tick. */
typedef struct
{
    navigation_command_t command;
    int requested_speed_percent;
} navigation_request_t;

/* What Sensor Monitor reports THIS tick - everything Safety needs to
   make a decision, gathered from all sensors in one place. */
typedef struct
{
    float            ultrasonic_cm[2];
    sensor_health_t  ultrasonic_health[2];

    lane_position_t  lane_position;
    sensor_health_t  lane_health;

    float            left_rpm;
    float            right_rpm;
    sensor_health_t  encoder_health;

    float            accel_x, accel_y, accel_z;   /* g */
    sensor_health_t  imu_health;

    /* CLOCK_MONOTONIC ms timestamp of when Sensor Monitor finished
       building THIS snapshot. Safety uses this to tell a fresh
       snapshot apart from one that's gone stale because Sensor
       Monitor itself has slowed down or died - see
       SENSOR_SNAPSHOT_STALE_MS in config.h. Not wall-clock time;
       only ever compared against another CLOCK_MONOTONIC read. */
    uint64_t         snapshot_time_ms;
} sensor_snapshot_t;

/* Envelope sent to Safety's channel - either a navigation_request_t
   (from Navigation) or a sensor_snapshot_t (from Sensor Monitor),
   distinguished by type. */
typedef struct
{
    message_type_t type;
    union
    {
        navigation_request_t navigation;
        sensor_snapshot_t    sensor;
    };
} safety_message_t;

typedef struct
{
    safety_state_t   state;
    int              approved_speed_percent;
    float            processing_time_ms;
    int              deadline_bound_ms;
    sensor_health_t  ultrasonic_health[2];
    lane_position_t  lane_position;
    sensor_health_t  lane_health;
} safety_reply_t;

/* Envelope sent to Sensor Monitor's channel - nothing sends it
   anything yet, but MSG_SYSTEM/MSG_HEARTBEAT stay reserved here for
   consistency with Safety's channel. */

/* What Safety commands Motor Controller to actually do. Direction is
   per-wheel (matches drv8833's phase/enable wiring - shared speed,
   independent direction); Safety has already made every safety
   decision by the time this is sent, so Motor Controller does not
   re-evaluate anything - it only executes. */
typedef struct
{
    wheel_dir_t   left_dir;
    wheel_dir_t   right_dir;
    unsigned int  speed_percent;
} motor_command_t;

typedef struct
{
    int accepted;
} motor_reply_t;

/* Lightweight ack for MSG_SENSOR_DATA - Sensor Monitor just needs to
   be unblocked, it doesn't need a decision back. */
typedef struct
{
    int ok;
} ack_reply_t;

#endif /* MESSAGES_H */
