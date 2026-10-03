/*
 * sensor_health.h - turns noisy raw readings of ONE ultrasonic sensor
 * into a filtered distance plus a trust status (see sensor_health.c)
 */
#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include "../common/messages.h"
#include "../sensors/median_filter.h"

/* Everything we remember about one sensor. */
typedef struct {
    median_filter_t filter;       /* last good readings                */
    sensor_status_t status;       /* WAITING / OK / FAULT              */
    int             bad_count;    /* consecutive bad readings          */
    int             ever_ok;      /* 1 once the sensor worked           */
    float           distance_cm;  /* current median (valid if OK)      */
} sensor_health_t;

/* Something worth logging happened on this update. */
typedef enum {
    SENSOR_EVENT_NONE = 0,
    SENSOR_EVENT_READY,       /* trusted for the first time          */
    SENSOR_EVENT_FAULT,       /* just declared faulty                */
    SENSOR_EVENT_RECOVERED    /* trusted again after a fault         */
} sensor_event_t;

/* Reset to "no data yet" (status WAITING). */
void sensor_health_init(sensor_health_t *sh);

/* Feed ONE raw reading (cm, < 0 or out of range = bad reading).
 * Updates filter/status/distance and reports a log-worthy event. */
sensor_event_t sensor_health_update(sensor_health_t *sh, float raw_cm);

#endif
