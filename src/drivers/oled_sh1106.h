/*
 * oled_sh1106.h - minimal QNX driver for a 128x64 SH1106 OLED on I2C
 * (see oled_sh1106.c)
 *
 * How it works: the driver keeps a copy of the screen in RAM (the
 * "framebuffer"). oled_clear() / oled_print() / oled_print_big() only
 * change that copy; nothing goes over I2C until oled_flush(). So a
 * whole new screen costs exactly ONE flush.
 *
 * Text rules: the built-in 5x7 font has digits, A-Z (lower case is
 * shown as upper case) and  . : - % /   Any other character is blank.
 */
#ifndef OLED_SH1106_H
#define OLED_SH1106_H

#include <stdint.h>

#define OLED_WIDTH       128
#define OLED_HEIGHT      64
#define OLED_PAGES       8          /* one page = 8 pixel rows           */
#define OLED_I2C_ADDR    0x3C

/* SH1106 RAM is 132 columns wide but the glass shows 128 of them,
 * starting at column 2. If the picture looks shifted by 2 pixels or
 * has a stray line at an edge, change this (0 or 2). */
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET 2
#endif

/* Set to 1 if the picture appears upside down on your module. */
#ifndef OLED_ROTATE_180
#define OLED_ROTATE_180  0
#endif

/* Opens the I2C bus device (e.g. "/dev/i2c1"), sends the SH1106 start-up
 * sequence, clears the screen. Returns 0 on success, -1 on failure. */
int  oled_init(const char *i2c_device);

/* Turns the panel off and closes the I2C device. */
void oled_close(void);

/* Erases the framebuffer (RAM only - call oled_flush() to show it). */
void oled_clear(void);

/* Small text: 6 pixels per character, 21 characters per line.
 * x = pixel column (0..127), page = text line (0..7). RAM only. */
void oled_print(int x, int page, const char *text);

/* BIG text: every letter 2x wider and 2x taller (12 px per character),
 * centred, using two pages: `page` and `page`+1. RAM only. */
void oled_print_big(int page, const char *text);

/* Sends the whole framebuffer to the panel over I2C. */
int  oled_flush(void);

#endif /* OLED_SH1106_H */
