/*
 * display_task.c
 * ---------------------------------------------------------------
 * Shows the two ultrasonic distances and a BIG status word on the OLED.
 * The actual panel access is in drivers/oled_sh1106.c; this file only
 * decides WHAT to show and WHEN.
 *
 * Design rules (QNX real-time thinking):
 *   - The display is the LEAST important task, so it has the lowest
 *     priority (PRIO_DISPLAY).
 *   - Safety talks to it with a PULSE (MsgSendPulse): non-blocking,
 *     fire-and-forget. Safety never waits for the OLED, so a slow I2C
 *     bus can never make a safety decision late.
 *   - All OLED/I2C calls happen ONLY in this thread.
 */
#include <stdio.h>
#include <errno.h>
#include <sys/neutrino.h>
#include "../display/display_task.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"
#include "../drivers/oled_sh1106.h"

#define BIG_TEXT_PAGE   4      /* big status word uses OLED pages 4 and 5 */

static int display_chid = -1;

/* display_get_chid: Safety uses this to attach to our channel. */
int display_get_chid(void)
{
    return display_chid;
}

/* state_text: the big word for each safety state. */
static const char *state_text(int state)
{
    switch (state) {
    case STATE_SAFE:      return "SAFE";
    case STATE_WARNING:   return "WARNING";
    case STATE_EMERGENCY: return "EMERGENCY";
    default:              return "FAULT";
    }
}

/* distance_text: "123.4" or "---" when the sensor has no trusted value. */
static void distance_text(char *out, size_t n, int tenths)
{
    if (tenths == DISPLAY_NO_VALUE)
        snprintf(out, n, "---");
    else
        snprintf(out, n, "%d.%d", tenths / 10, tenths % 10);
}

/* draw_screen: build one complete frame from the packed pulse value in
 * the RAM framebuffer, then send it to the panel with ONE oled_flush(). */
static void draw_screen(int packed)
{
    char d0[16], d1[16], line[24];

    distance_text(d0, sizeof(d0), (packed >> 2)  & 0xFFF);
    distance_text(d1, sizeof(d1), (packed >> 14) & 0xFFF);

    oled_clear();

    snprintf(line, sizeof(line), "U1: %s CM", d0);
    oled_print(0, 0, line);
    snprintf(line, sizeof(line), "U2: %s CM", d1);
    oled_print(0, 1, line);

    oled_print_big(BIG_TEXT_PAGE, state_text(packed & 0x3));

    oled_flush();
}

/* display_thread: waits for pulses from Safety and redraws.
 *   - If the OLED could not be opened we keep receiving (and
 *     ignoring) pulses, so the kernel's pulse queue never fills up.
 *   - A frame identical to the last one is skipped: every redraw is a
 *     full I2C transfer and there is no point repeating it.          */
static void *display_thread(void *arg)
{
    struct _pulse pulse;
    int oled_ok = (oled_init(OLED_I2C_DEVICE) == 0);
    int last_drawn = -1;
    (void)arg;

    if (!oled_ok)
        log_msg("[DISPLAY] OLED not available on %s - running without screen",
                OLED_I2C_DEVICE);

    for (;;) {
        int rcvid = MsgReceive(display_chid, &pulse, sizeof(pulse), NULL);

        if (rcvid != 0 || pulse.code != PULSE_DISPLAY)
            continue;                         /* only pulses are expected */
        if (!oled_ok)
            continue;

        if (pulse.value.sival_int != last_drawn) {
            last_drawn = pulse.value.sival_int;
            draw_screen(last_drawn);
        }
    }
    return NULL;
}

/* display_start: channel first (Safety attaches to it later), then
 * the thread. */
int display_start(void)
{
    display_chid = ChannelCreate(0);
    if (display_chid == -1) {
        perror("display: ChannelCreate");
        return -1;
    }
    return create_rt_thread(display_thread, NULL, PRIO_DISPLAY, "display");
}
