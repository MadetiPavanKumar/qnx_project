#include "f429_encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <rpi_gpio.h>

int f429_encoder_init(
        f429_encoder_t *encoder,
        int gpio_pin,
        uint32_t ppr,
        uint32_t measurement_time_ms,
        uint32_t poll_interval_us)
{
    unsigned level;

    if (encoder == NULL)
    {
        fprintf(stderr, "F429: NULL encoder pointer\n");
        return -1;
    }

    if (ppr == 0)
    {
        fprintf(stderr, "F429: Invalid PPR\n");
        return -1;
    }

    if (measurement_time_ms == 0)
    {
        fprintf(stderr, "F429: Invalid measurement time\n");
        return -1;
    }

    if (poll_interval_us == 0)
    {
        fprintf(stderr, "F429: Invalid polling interval\n");
        return -1;
    }

    /*
     * Configure GPIO as input.
     */
    if (rpi_gpio_setup(gpio_pin, GPIO_IN) != 0)
    {
        perror("F429: GPIO setup failed");
        return -1;
    }

    /*
     * Read initial GPIO state.
     */
    if (rpi_gpio_input(gpio_pin, &level) != 0)
    {
        fprintf(stderr, "F429: Initial GPIO read failed\n");
        return -1;
    }

    encoder->gpio_pin = gpio_pin;
    encoder->pulses_per_revolution = ppr;
    encoder->measurement_time_ms = measurement_time_ms;
    encoder->poll_interval_us = poll_interval_us;

    encoder->previous_state = level;

    encoder->pulse_count = 0;
    encoder->rpm = 0.0f;

    encoder->initialized = 1;

    return 0;
}


int f429_encoder_measure_rpm(
        f429_encoder_t *encoder)
{
    unsigned level;
    unsigned current_state;

    if (encoder == NULL)
    {
        return -1;
    }

    if (!encoder->initialized)
    {
        fprintf(stderr, "F429: Driver not initialized\n");
        return -1;
    }

    encoder->pulse_count = 0;

    /*
     * Measure for the configured time.
     */
    uint32_t samples =
        encoder->measurement_time_ms;

    for (uint32_t i = 0; i < samples; i++)
    {
        /*
         * Read GPIO.
         */
        if (rpi_gpio_input(
                encoder->gpio_pin,
                &level) != 0)
        {
            fprintf(stderr, "F429: GPIO read failed\n");
            return -1;
        }

        current_state = level;

        /*
         * Detect LOW -> HIGH transition.
         */
        if ((encoder->previous_state == GPIO_LOW) &&
            (current_state == GPIO_HIGH))
        {
            encoder->pulse_count++;
        }

        encoder->previous_state = current_state;

        /*
         * Wait for next sample.
         */
        usleep(encoder->poll_interval_us);
    }

    /*
     * RPM calculation.
     *
     * RPM =
     * pulses / PPR
     * multiplied by
     * 60 / measurement_time_seconds
     */
    float measurement_seconds =
        (float)encoder->measurement_time_ms / 1000.0f;

    encoder->rpm =
        ((float)encoder->pulse_count /
         (float)encoder->pulses_per_revolution)
        * (60.0f / measurement_seconds);

    return 0;
}


float f429_encoder_get_rpm(
        const f429_encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return 0.0f;
    }

    return encoder->rpm;
}


uint32_t f429_encoder_get_pulses(
        const f429_encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    return encoder->pulse_count;
}


void f429_encoder_deinit(
        f429_encoder_t *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->initialized = 0;
    encoder->pulse_count = 0;
    encoder->rpm = 0.0f;
}
