#include <stdio.h>
#include "hcsr04_driver.h"
#include "hc_sr04.h"
#include "../common/gpio_map.h"

static int handles[2] = { -1, -1 };
static int driver_ready = 0;

int hcsr04_driver_init(void)
{
    handles[0] = hc_sr04_begin(GPIO_HCSR04_0_TRIG, GPIO_HCSR04_0_ECHO);
    handles[1] = hc_sr04_begin(GPIO_HCSR04_1_TRIG, GPIO_HCSR04_1_ECHO);

    if (handles[0] < 0 || handles[1] < 0)
    {
        fprintf(stderr,
                "hcsr04_driver_init: hc_sr04_begin failed (h0=%d h1=%d)\n",
                handles[0], handles[1]);
        return -1;
    }

    driver_ready = 1;
    return 0;
}

float hcsr04_read_raw_cm(int sensor_index)
{
    if (!driver_ready || sensor_index < 0 || sensor_index > 1)
        return -1.0f;

    /* read_distance() paces itself against HC_SR04_DEFAULT_MEASUREMENT_
       INTERVAL_US (65ms) since THIS sensor's own last read, and returns
       -1.0f on no-echo/timeout/out-of-range - sensor_health.c already
       treats that as a bad sample rather than "far away." */
    return read_distance(handles[sensor_index]);
}

void hcsr04_driver_deinit(void)
{
    if (handles[0] >= 0) hc_sr04_end(handles[0]);
    if (handles[1] >= 0) hc_sr04_end(handles[1]);
    driver_ready = 0;
}
