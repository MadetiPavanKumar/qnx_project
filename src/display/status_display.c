#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "status_display.h"
#include "oled.h"
#include "../common/labels.h"

static int display_ready = 0;

/* 128px wide / 6px per character cell (5px glyph + 1px gap) = ~21
   characters per row. Every row is padded to this width so a shorter
   new string fully overwrites a longer old one - the OLED buffer
   isn't cleared between updates, only at init. */
#define ROW_WIDTH_CHARS 21

static void print_row(int row, const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    /* Pad with spaces to ROW_WIDTH_CHARS. */
    int len = strlen(buf);
    for (int i = len; i < ROW_WIDTH_CHARS && i < (int)sizeof(buf) - 1; i++)
        buf[i] = ' ';
    buf[ROW_WIDTH_CHARS < (int)sizeof(buf) - 1 ? ROW_WIDTH_CHARS : (int)sizeof(buf) - 1] = '\0';

    oled_set_cursor(0, row);
    oled_print(buf);
}

int status_display_init(void)
{
    if (oled_init("/dev/i2c1") != 0)
    {
        fprintf(stderr, "status_display_init: OLED not available, "
                        "continuing without it\n");
        return -1;
    }

    oled_clear();
    oled_set_cursor(0, 0);
    oled_print("AV SAFETY SUPERVISOR");
    display_ready = 1;
    return 0;
}

void status_display_update(const telemetry_t *t)
{
    if (!display_ready)
        return;

    print_row(1, "D0:%.0fcm D1:%.0fcm", t->ultrasonic_cm[0], t->ultrasonic_cm[1]);
    print_row(2, "ST:%s SPD:%d%%", safety_state_label(t->state), t->approved_speed_percent);
    print_row(3, "LANE:%s", lane_position_label(t->lane_position));
    print_row(4, "RPM L:%.0f R:%.0f", t->left_rpm, t->right_rpm);
    print_row(5, "AX:%.1f AY:%.1f AZ:%.1f", t->accel_x, t->accel_y, t->accel_z);

    /* Compact health line: U0/U1/LANE/ENCODER/IMU, one letter each -
       O=OK, D=DEGRADED, F=FAULT. */
    char h[6];
    h[0] = sensor_health_label(t->ultrasonic_health[0])[0];
    h[1] = sensor_health_label(t->ultrasonic_health[1])[0];
    h[2] = sensor_health_label(t->lane_health)[0];
    h[3] = sensor_health_label(t->encoder_health)[0];
    h[4] = sensor_health_label(t->imu_health)[0];
    h[5] = '\0';
    print_row(6, "HLTH U0U1LN EN IM:%s", h);

    print_row(7, "ESTOP:%s", t->manual_estop_active ? "ACTIVE!!" : "off");
}
