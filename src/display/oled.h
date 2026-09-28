#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* OLED configuration */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_I2C_ADDR   0x3C

/* Initialize OLED */
int oled_init(const char *i2c_device);

/* Clear entire display */
void oled_clear(void);

/* Update display buffer */
void oled_update(void);

/* Set text cursor
 * x = column (0-127)
 * page = row (0-7)
 */
void oled_set_cursor(uint8_t x, uint8_t page);

/* Print string */
void oled_print(const char *text);

/* Print integer */
void oled_print_int(int value);

/* Print floating-point number */
void oled_print_float(float value, int decimals);

/* Turn display on/off */
void oled_display_on(void);
void oled_display_off(void);

/* Close OLED */
void oled_close(void);

#endif
