#include <stdio.h>
#include "line_position.h"
#include "../drivers/ir_array_driver.h"
#include "../common/gpio_map.h"

static int driver_ready = 0;

int line_position_init(void)
{
    IR_Config config;

    config.gpio[0] = GPIO_IR_CH1;
    config.gpio[1] = GPIO_IR_CH2;
    config.gpio[2] = GPIO_IR_CH3;
    config.gpio[3] = GPIO_IR_CH4;
    config.gpio[4] = GPIO_IR_CH5;

    /* Starting at 0 (HIGH = line detected), matching your own
       ir_5ch_test.c comment: this hasn't been confirmed for your
       specific TCRT5000L modules yet. Run the standalone test, watch
       the printed sensor[] values against a real light/dark surface,
       and flip this to 1 if the logic reads backwards. */
    config.active_low = 0;

    if (IRArray_Init(&config) != IR_SUCCESS)
    {
        fprintf(stderr, "line_position_init: IRArray_Init failed\n");
        return -1;
    }

    driver_ready = 1;
    return 0;
}

/* Translates the driver's IR_Position into our own lane_position_t.
   Kept as an explicit table rather than assuming the enum orderings
   match - if either enum's order ever changes, this is the one place
   that needs updating, not every caller. */
static lane_position_t translate_position(IR_Position p)
{
    switch (p)
    {
        case IR_POSITION_FAR_LEFT:     return LANE_FAR_LEFT;
        case IR_POSITION_LEFT:         return LANE_LEFT;
        case IR_POSITION_SLIGHT_LEFT:  return LANE_SLIGHT_LEFT;
        case IR_POSITION_CENTER:       return LANE_CENTER;
        case IR_POSITION_SLIGHT_RIGHT: return LANE_SLIGHT_RIGHT;
        case IR_POSITION_RIGHT:        return LANE_RIGHT;
        case IR_POSITION_FAR_RIGHT:    return LANE_FAR_RIGHT;
        case IR_POSITION_LOST:         return LANE_LOST;
        case IR_POSITION_UNKNOWN:
        default:                        return LANE_UNKNOWN;
    }
}

void line_position_read(lane_position_t *out_position, sensor_health_t *out_health)
{
    IR_Reading reading;

    if (!driver_ready)
    {
        *out_position = LANE_UNKNOWN;
        *out_health   = SENSOR_FAULT;
        return;
    }

    int rc = IRArray_Read(&reading);

    if (rc != IR_SUCCESS || !reading.valid)
    {
        /* Driver already tracks consecutive failures internally and
           flips its own healthy flag after IR_MAX_READ_FAILURES -
           reuse that judgment rather than re-counting here. */
        *out_position = LANE_UNKNOWN;
        *out_health   = IRArray_IsHealthy() ? SENSOR_DEGRADED : SENSOR_FAULT;
        return;
    }

    *out_position = translate_position(reading.position);
    *out_health   = IRArray_IsHealthy() ? SENSOR_OK : SENSOR_FAULT;
}

void line_position_deinit(void)
{
    IRArray_Close();
    driver_ready = 0;
}
