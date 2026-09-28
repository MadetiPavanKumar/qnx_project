#ifndef HC_SR04_H
#define HC_SR04_H

/*
 * HC-SR04 QNX Driver
 *
 * Raspberry Pi 4 / BCM2711
 *
 * Default:
 *   TRIG = GPIO5
 *   ECHO = GPIO6
 *
 * Designed for:
 *   QNX Neutrino
 *   Momentics
 *
 * Measurement method:
 *   Direct GPIO MMIO polling
 *   ClockCycles() high-resolution timing
 */

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * DEFAULT CONFIGURATION
 * ============================================================ */

#define HC_SR04_DEFAULT_TRIG_GPIO              5
#define HC_SR04_DEFAULT_ECHO_GPIO              6

#define HC_SR04_DEFAULT_MIN_DISTANCE_CM        2.0f
#define HC_SR04_DEFAULT_MAX_DISTANCE_CM        200.0f

/*
 * Speed of sound at approximately room temperature.
 *
 * 343 m/s = 0.0343 cm/us
 */
#define HC_SR04_DEFAULT_SPEED_OF_SOUND         0.0343f

/*
 * Minimum time between measurements.
 *
 * 65 ms is deliberately conservative and prevents
 * ultrasonic cross-talk / ringing between measurements.
 */
#define HC_SR04_DEFAULT_MEASUREMENT_INTERVAL_US 65000ULL


/*
 * Maximum time allowed while waiting for ECHO to become HIGH.
 *
 * 30 ms gives considerable scheduling margin.
 */
#define HC_SR04_DEFAULT_RISING_TIMEOUT_US      30000ULL


/*
 * Maximum valid ECHO HIGH time.
 *
 * At 200 cm:
 *
 *     echo ~= 11662 us
 *
 * 15000 us gives margin while still rejecting
 * obviously invalid/stuck-high signals.
 */
#define HC_SR04_DEFAULT_FALLING_TIMEOUT_US     15000ULL


/*
 * Valid pulse limits.
 *
 * 2 cm:
 *     approximately 116 us
 *
 * 200 cm:
 *     approximately 11662 us
 */
#define HC_SR04_DEFAULT_MIN_PULSE_US           80ULL
#define HC_SR04_DEFAULT_MAX_PULSE_US           12000ULL


/*
 * Maximum time allowed for ECHO recovery if it gets stuck HIGH.
 */
#define HC_SR04_ECHO_RECOVERY_TIMEOUT_US       5000ULL


/* ============================================================
 * STATUS
 * ============================================================ */

typedef enum
{
    HC_SR04_OK = 0,

    /*
     * Trigger completed but no ECHO rising edge was received.
     */
    HC_SR04_NO_ECHO,

    /*
     * ECHO went HIGH but stayed HIGH too long.
     */
    HC_SR04_ECHO_TIMEOUT,

    /*
     * Echo pulse was shorter than physically expected.
     */
    HC_SR04_TOO_CLOSE,

    /*
     * Echo pulse was outside configured range.
     */
    HC_SR04_OUT_OF_RANGE,

    /*
     * Driver initialization failure.
     */
    HC_SR04_INIT_ERROR,

    /*
     * Driver has not been initialized.
     */
    HC_SR04_NOT_INITIALIZED,

    /*
     * Configuration is invalid.
     */
    HC_SR04_INVALID_CONFIG,

    /*
     * Internal/system error.
     */
    HC_SR04_SYSTEM_ERROR

} hc_sr04_status_t;


/* ============================================================
 * CONFIGURATION
 * ============================================================ */

typedef struct
{
    /*
     * BCM GPIO numbers.
     */
    unsigned trig_gpio;
    unsigned echo_gpio;


    /*
     * Valid distance range.
     */
    float min_distance_cm;
    float max_distance_cm;


    /*
     * Speed of sound in cm/us.
     *
     * Default = 0.0343
     */
    float speed_of_sound_cm_per_us;


    /*
     * Time between measurements.
     */
    uint64_t measurement_interval_us;


    /*
     * Timeout waiting for ECHO HIGH.
     */
    uint64_t rising_timeout_us;


    /*
     * Timeout waiting for ECHO LOW.
     */
    uint64_t falling_timeout_us;


    /*
     * Minimum accepted pulse width.
     */
    uint64_t min_pulse_us;


    /*
     * Maximum accepted pulse width.
     */
    uint64_t max_pulse_us;


} hc_sr04_config_t;


/* ============================================================
 * STATISTICS
 * ============================================================ */

typedef struct
{
    uint64_t total_measurements;

    uint64_t successful_measurements;

    uint64_t no_echo_count;

    uint64_t echo_timeout_count;

    uint64_t too_close_count;

    uint64_t out_of_range_count;

    uint64_t system_error_count;


    /*
     * Last valid result.
     */
    float last_distance_cm;

    uint64_t last_echo_us;


    /*
     * Statistics over valid measurements.
     */
    float minimum_distance_cm;

    float maximum_distance_cm;

} hc_sr04_stats_t;


/* ============================================================
 * DRIVER OBJECT
 * ============================================================ */

typedef struct
{
    hc_sr04_config_t cfg;

    hc_sr04_stats_t stats;


    /*
     * GPIO MMIO base.
     */
    volatile uint32_t *gpio_base;


    /*
     * Clock frequency used by ClockCycles().
     */
    uint64_t cycles_per_sec;


    /*
     * Initialization state.
     */
    bool initialized;


    /*
     * Protects the measurement operation.
     *
     * This makes the driver safe if several application
     * threads attempt to access the same sensor.
     */
    pthread_mutex_t mutex;


} hc_sr04_t;


/* ============================================================
 * API
 * ============================================================ */


/*
 * Fill configuration structure with recommended defaults.
 */
void hc_sr04_config_default(hc_sr04_config_t *config);


/*
 * Initialize sensor.
 *
 * Returns:
 *   HC_SR04_OK
 *   HC_SR04_INVALID_CONFIG
 *   HC_SR04_INIT_ERROR
 */
hc_sr04_status_t hc_sr04_init(
        hc_sr04_t *sensor,
        const hc_sr04_config_t *config);


/*
 * Deinitialize sensor.
 */
void hc_sr04_deinit(hc_sr04_t *sensor);


/*
 * Perform one complete measurement.
 *
 * distance_cm:
 *      Valid distance if return == HC_SR04_OK.
 *
 * echo_us:
 *      Raw ECHO pulse width.
 */
hc_sr04_status_t hc_sr04_read(
        hc_sr04_t *sensor,
        float *distance_cm,
        uint64_t *echo_us);


/*
 * Convenience function.
 *
 * Returns distance directly.
 *
 * If measurement fails:
 *      returns -1.0f
 */
float hc_sr04_read_distance(
        hc_sr04_t *sensor);


/*
 * Wait the configured measurement interval.
 *
 * Useful for continuous measurement loops.
 */
void hc_sr04_wait_next_measurement(
        hc_sr04_t *sensor);


/*
 * Convert status to readable string.
 */
const char *hc_sr04_status_string(
        hc_sr04_status_t status);


/*
 * Get statistics.
 */
void hc_sr04_get_stats(
        hc_sr04_t *sensor,
        hc_sr04_stats_t *stats);


/*
 * Reset statistics.
 */
void hc_sr04_reset_stats(
        hc_sr04_t *sensor);


/*
 * Check whether distance is valid.
 */
bool hc_sr04_distance_valid(
        const hc_sr04_t *sensor,
        float distance_cm);


/*
 * Try to configure the calling thread for real-time execution.
 *
 * This is optional.
 *
 * Returns:
 *      0  success
 *     -1  failed
 */
int hc_sr04_enable_realtime(void);


/* ============================================================
 * SIMPLE API
 * ============================================================
 *
 * Minimal interface for callers who just want distance readings
 * without touching hc_sr04_t / hc_sr04_config_t directly.
 *
 * Everything (defaults, timing, validation, cross-talk spacing)
 * is still handled inside the driver -- you only supply the
 * TRIG / ECHO GPIO numbers per sensor.
 *
 * Supports up to HC_SR04_SIMPLE_MAX_SENSORS sensors at once, each
 * identified by the handle returned from hc_sr04_begin().
 *
 * Usage (one sensor):
 *
 *     int s1 = hc_sr04_begin(5, 6);
 *
 *     float distance = read_distance(s1);
 *
 *     hc_sr04_end(s1);
 *
 * Usage (two sensors):
 *
 *     int s1 = hc_sr04_begin(5, 6);
 *     int s2 = hc_sr04_begin(17, 27);
 *
 *     float d1 = read_distance(s1);
 *     float d2 = read_distance(s2);
 *
 *     hc_sr04_end(s1);
 *     hc_sr04_end(s2);
 *
 * For control over configuration (custom ranges, timeouts, etc.)
 * use the full API above (hc_sr04_init / hc_sr04_read / ...)
 * instead.
 */

#define HC_SR04_SIMPLE_MAX_SENSORS     4


/*
 * Initialize a sensor on the given GPIO pins, using all other
 * driver defaults (see HC_SR04_DEFAULT_* in this header).
 *
 * Returns:
 *   A handle (>= 0) to pass to read_distance() / hc_sr04_end(),
 *   on success.
 *
 *   -1 if initialization failed, or if
 *   HC_SR04_SIMPLE_MAX_SENSORS sensors are already active.
 */
int hc_sr04_begin(
        unsigned trig_gpio,
        unsigned echo_gpio);


/*
 * Perform one measurement on the sensor identified by handle and
 * return the distance in cm.
 *
 * Automatically waits out that sensor's configured measurement
 * interval since its previous read, so it is safe to call this
 * back to back in a loop. Reads on different handles are
 * independent of each other.
 *
 * Returns -1.0f if handle is invalid/not active, or if the
 * measurement failed (no echo, out of range, etc.).
 */
float read_distance(int handle);


/*
 * Shut down the sensor identified by handle, freeing its slot for
 * reuse by a future hc_sr04_begin() call.
 */
void hc_sr04_end(int handle);


#ifdef __cplusplus
}
#endif

#endif
