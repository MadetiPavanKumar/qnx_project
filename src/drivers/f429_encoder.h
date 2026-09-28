#ifndef F429_ENCODER_H
#define F429_ENCODER_H

#include <stdint.h>

#define F429_ENCODER_DEFAULT_PPR             20
#define F429_ENCODER_DEFAULT_MEASUREMENT_MS 1000
#define F429_ENCODER_DEFAULT_POLL_US        1000

typedef struct
{
    int gpio_pin;
    uint32_t pulses_per_revolution;
    uint32_t measurement_time_ms;
    uint32_t poll_interval_us;

    unsigned previous_state;

    uint32_t pulse_count;
    float rpm;

    int initialized;

} f429_encoder_t;

/*
 * Initialize the F429 encoder.
 *
 * gpio_pin:
 *      GPIO connected to the F429 output.
 *
 * ppr:
 *      Pulses per revolution.
 *      Use 20 for your current encoder disc.
 *
 * measurement_time_ms:
 *      RPM measurement window.
 *
 * poll_interval_us:
 *      GPIO polling interval.
 *
 * Returns:
 *      0  = success
 *     -1  = failure
 */
int f429_encoder_init(
        f429_encoder_t *encoder,
        int gpio_pin,
        uint32_t ppr,
        uint32_t measurement_time_ms,
        uint32_t poll_interval_us);

/*
 * Measure RPM.
 *
 * This function blocks for measurement_time_ms.
 *
 * Returns:
 *      0  = success
 *     -1  = GPIO read failure
 */
int f429_encoder_measure_rpm(
        f429_encoder_t *encoder);

/*
 * Get the RPM from the most recent measurement.
 */
float f429_encoder_get_rpm(
        const f429_encoder_t *encoder);

/*
 * Get the pulse count from the most recent measurement.
 */
uint32_t f429_encoder_get_pulses(
        const f429_encoder_t *encoder);

/*
 * Stop/deinitialize the encoder driver.
 */
void f429_encoder_deinit(
        f429_encoder_t *encoder);

#endif
