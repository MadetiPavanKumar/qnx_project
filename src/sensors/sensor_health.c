/*
 * sensor_health.c
 * ---------------------------------------------------------------
 * This is where the MEDIAN FILTER (median_filter.c) is actually used.
 * One sensor_health_t exists per ultrasonic sensor and lives inside
 * the Sensor Monitor task (sensor_monitor.c), which calls
 * sensor_health_update() once for every raw reading it takes.
 *
 * Rules (the part that stops "fault / recover / fault" flapping):
 *   GOOD reading (inside MIN_VALID_CM..MAX_VALID_CM):
 *      - bad counter back to 0
 *      - reading goes into the median window
 *      - once the window holds MEDIAN_MIN_SAMPLES readings the sensor
 *        is trusted (status OK) and distance = median of the window
 *   BAD reading (-1 timeout / no echo / garbage):
 *      - NOT stored, the previous median simply stays in use
 *      - SENSOR_FAULT_AFTER bad readings IN A ROW -> status FAULT
 *      - while the sensor is not OK, a bad reading empties the window,
 *        so recovery needs MEDIAN_MIN_SAMPLES good readings IN A ROW
 */
#include "../sensors/sensor_health.h"
#include "../common/config.h"

/* sensor_health_init: start every sensor "not trusted" with an empty
 * window - fail-safe from the very first tick. */
void sensor_health_init(sensor_health_t *sh)
{
    mf_init(&sh->filter);
    sh->status      = SENSOR_WAITING;
    sh->bad_count   = 0;
    sh->ever_ok     = 0;
    sh->distance_cm = MAX_VALID_CM;
}

/* sensor_health_update: process one raw reading (see rules above).
 * Returns an event so the CALLER can log it - this file does no I/O. */
sensor_event_t sensor_health_update(sensor_health_t *sh, float raw_cm)
{
    sensor_event_t event = SENSOR_EVENT_NONE;

    if (raw_cm >= MIN_VALID_CM && raw_cm <= MAX_VALID_CM) {
        /* ---- good reading ---- */
        sh->bad_count = 0;
        mf_add(&sh->filter, raw_cm);

        if (sh->status != SENSOR_OK && mf_ready(&sh->filter)) {
            event = sh->ever_ok ? SENSOR_EVENT_RECOVERED : SENSOR_EVENT_READY;
            sh->status  = SENSOR_OK;
            sh->ever_ok = 1;
        }
        if (sh->status == SENSOR_OK)
            sh->distance_cm = mf_median(&sh->filter);
    } else {
        /* ---- bad reading: ignore it, just count it ---- */
        if (sh->bad_count < SENSOR_FAULT_AFTER)
            sh->bad_count++;

        if (sh->status != SENSOR_FAULT && sh->bad_count >= SENSOR_FAULT_AFTER) {
            sh->status = SENSOR_FAULT;
            mf_init(&sh->filter);
            event = SENSOR_EVENT_FAULT;
        } else if (sh->status != SENSOR_OK) {
            mf_init(&sh->filter);       /* recovery must restart */
        }
    }
    return event;
}
