#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

typedef enum
{
    CMD_STOP,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT
} navigation_command_t;

/* Safety Supervisor's decision for the current tick. */
typedef enum
{
    SAFETY_SAFE,        /* requested speed approved as-is */
    SAFETY_WARNING,      /* speed reduced, still moving    */
    SAFETY_EMERGENCY,    /* hard stop - obstacle too close */
    SAFETY_FAULT         /* hard stop - a sensor is unreliable */
} safety_state_t;

/* Health of one ultrasonic sensor, tracked across ticks. */
typedef enum
{
    SENSOR_OK,
    SENSOR_DEGRADED,   /* some recent bad readings, still being used   */
    SENSOR_FAULT        /* too many bad readings, no longer trusted     */
} sensor_health_t;

/* Where the robot is relative to the line, as reported by the 5-
   channel IR array. This is OUR project's own enum, kept separate
   from the driver's IR_Position - line_position.c is the only file
   that translates between them, so a future IR hardware/driver swap
   never touches safety_supervisor.c or messages.h. */
typedef enum
{
    LANE_UNKNOWN = 0,
    LANE_FAR_LEFT,
    LANE_LEFT,
    LANE_SLIGHT_LEFT,
    LANE_CENTER,
    LANE_SLIGHT_RIGHT,
    LANE_RIGHT,
    LANE_FAR_RIGHT,
    LANE_LOST           /* no channel sees the line at all */
} lane_position_t;

/* Per-wheel direction, as SAFETY decides it. Independent of
   drv8833_dir_t (the driver's own type) - motor_controller.c is the
   only file that translates between them, same reasoning as
   lane_position_t vs IR_Position. */
typedef enum
{
    WHEEL_STOP = 0,
    WHEEL_FORWARD,
    WHEEL_REVERSE
} wheel_dir_t;

#endif /* SYSTEM_TYPES_H */
