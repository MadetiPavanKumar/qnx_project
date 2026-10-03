/*
 * display_task.h - OLED status display task (see display_task.c)
 *
 * Screen layout (128x64):
 *
 *     U1: 123.4 CM
 *     U2: 118.0 CM
 *
 *       EMERGENCY          <- big text: SAFE / WARNING / EMERGENCY / FAULT
 */
#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include <stdint.h>
#include "../common/messages.h"

/* Creates the display channel and starts the (low priority) thread.
 * The thread never makes start-up fail: with no OLED connected the
 * system simply runs without a screen. */
int display_start(void);

/* Channel id - Safety attaches to it to send display pulses. */
int display_get_chid(void);

/* A pulse can only carry ONE 32-bit number, so Safety packs
 *   bits  0..1  : safety state
 *   bits  2..13 : sensor 1 distance in 0.1 cm (4095 = no valid value)
 *   bits 14..25 : sensor 2 distance in 0.1 cm (4095 = no valid value)
 * and display_task.c unpacks it. A negative distance means "no valid
 * value" (sensor not trusted). */
#define DISPLAY_NO_VALUE   4095

static inline int display_tenths(float cm)
{
    int t = (int)(cm * 10.0f + 0.5f);
    if (cm < 0.0f || t >= DISPLAY_NO_VALUE)
        return DISPLAY_NO_VALUE;
    return t;
}

static inline int display_pack(safety_state_t state, float d0_cm, float d1_cm)
{
    return (int)state | (display_tenths(d0_cm) << 2) | (display_tenths(d1_cm) << 14);
}

#endif
