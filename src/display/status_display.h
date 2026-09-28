#ifndef STATUS_DISPLAY_H
#define STATUS_DISPLAY_H

#include "../system/telemetry.h"

int status_display_init(void);

/* Renders the full telemetry snapshot across all 8 OLED rows. */
void status_display_update(const telemetry_t *t);

#endif /* STATUS_DISPLAY_H */
