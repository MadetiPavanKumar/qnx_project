#include <stdio.h>
#include "encoder_driver.h"
#include "f429_encoder.h"
#include "../common/gpio_map.h"

/* Short measurement window per tick, not the driver's 1-second
   default - f429_encoder_measure_rpm() BLOCKS for this long, and we
   need it to fit inside one ~75-150ms tick alongside the ultrasonic
   reads. 30ms with a 300us poll interval gives ~100 samples per
   window, which is plenty of resolution at PPR=20 for anything but
   very slow crawl speeds. */
#define ENCODER_MEASUREMENT_MS   30
#define ENCODER_POLL_US         300

static f429_encoder_t left_enc;
static f429_encoder_t right_enc;
static int ready = 0;

int encoder_driver_init(void)
{
    if (f429_encoder_init(&left_enc, GPIO_ENCODER_LEFT,
                           F429_ENCODER_DEFAULT_PPR,
                           ENCODER_MEASUREMENT_MS, ENCODER_POLL_US) != 0)
    {
        fprintf(stderr, "encoder_driver_init: left encoder init failed\n");
        return -1;
    }

    if (f429_encoder_init(&right_enc, GPIO_ENCODER_RIGHT,
                           F429_ENCODER_DEFAULT_PPR,
                           ENCODER_MEASUREMENT_MS, ENCODER_POLL_US) != 0)
    {
        fprintf(stderr, "encoder_driver_init: right encoder init failed\n");
        return -1;
    }

    ready = 1;
    return 0;
}

void encoder_driver_read_rpm(float *left_rpm, float *right_rpm)
{
    if (!ready)
    {
        *left_rpm = 0.0f;
        *right_rpm = 0.0f;
        return;
    }

    /* Sequential: left window, then right window - together they add
       roughly 2 * ENCODER_MEASUREMENT_MS to this tick's duration. */
    f429_encoder_measure_rpm(&left_enc);
    f429_encoder_measure_rpm(&right_enc);

    *left_rpm  = f429_encoder_get_rpm(&left_enc);
    *right_rpm = f429_encoder_get_rpm(&right_enc);
}

void encoder_driver_deinit(void)
{
    f429_encoder_deinit(&left_enc);
    f429_encoder_deinit(&right_enc);
    ready = 0;
}
