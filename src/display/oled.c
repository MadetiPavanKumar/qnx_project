#include "oled.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>

/* ============================================================
 * Internal variables
 * ============================================================ */

static int oled_fd = -1;

static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

static uint8_t cursor_x = 0;
static uint8_t cursor_page = 0;


/* ============================================================
 * 5x7 Font
 *
 * Only basic ASCII characters are included.
 * Characters: space, numbers, A-Z, a-z
 * ============================================================ */

static const uint8_t font5x7[][5] =
{
    /* SPACE */
    {0x00,0x00,0x00,0x00,0x00},

    /* ! */
    {0x00,0x00,0x5F,0x00,0x00},

    /* " */
    {0x00,0x07,0x00,0x07,0x00},

    /* # */
    {0x14,0x7F,0x14,0x7F,0x14},

    /* $ */
    {0x24,0x2A,0x7F,0x2A,0x12},

    /* % */
    {0x23,0x13,0x08,0x64,0x62},

    /* & */
    {0x36,0x49,0x55,0x22,0x50},

    /* ' */
    {0x00,0x05,0x03,0x00,0x00},

    /* ( */
    {0x00,0x1C,0x22,0x41,0x00},

    /* ) */
    {0x00,0x41,0x22,0x1C,0x00},

    /* * */
    {0x14,0x08,0x3E,0x08,0x14},

    /* + */
    {0x08,0x08,0x3E,0x08,0x08},

    /* , */
    {0x00,0x50,0x30,0x00,0x00},

    /* - */
    {0x08,0x08,0x08,0x08,0x08},

    /* . */
    {0x00,0x60,0x60,0x00,0x00},

    /* / */
    {0x20,0x10,0x08,0x04,0x02},

    /* 0 */
    {0x3E,0x51,0x49,0x45,0x3E},

    /* 1 */
    {0x00,0x42,0x7F,0x40,0x00},

    /* 2 */
    {0x42,0x61,0x51,0x49,0x46},

    /* 3 */
    {0x21,0x41,0x45,0x4B,0x31},

    /* 4 */
    {0x18,0x14,0x12,0x7F,0x10},

    /* 5 */
    {0x27,0x45,0x45,0x45,0x39},

    /* 6 */
    {0x3C,0x4A,0x49,0x49,0x30},

    /* 7 */
    {0x01,0x71,0x09,0x05,0x03},

    /* 8 */
    {0x36,0x49,0x49,0x49,0x36},

    /* 9 */
    {0x06,0x49,0x49,0x29,0x1E},

    /* : */
    {0x00,0x36,0x36,0x00,0x00},

    /* ; */
    {0x00,0x56,0x36,0x00,0x00},

    /* < */
    {0x08,0x14,0x22,0x41,0x00},

    /* = */
    {0x14,0x14,0x14,0x14,0x14},

    /* > */
    {0x41,0x22,0x14,0x08,0x00},

    /* ? */
    {0x02,0x01,0x51,0x09,0x06},

    /* @ */
    {0x32,0x49,0x79,0x41,0x3E},

    /* A */
    {0x7E,0x11,0x11,0x11,0x7E},

    /* B */
    {0x7F,0x49,0x49,0x49,0x36},

    /* C */
    {0x3E,0x41,0x41,0x41,0x22},

    /* D */
    {0x7F,0x41,0x41,0x22,0x1C},

    /* E */
    {0x7F,0x49,0x49,0x49,0x41},

    /* F */
    {0x7F,0x09,0x09,0x09,0x01},

    /* G */
    {0x3E,0x41,0x49,0x49,0x7A},

    /* H */
    {0x7F,0x08,0x08,0x08,0x7F},

    /* I */
    {0x00,0x41,0x7F,0x41,0x00},

    /* J */
    {0x20,0x40,0x41,0x3F,0x01},

    /* K */
    {0x7F,0x08,0x14,0x22,0x41},

    /* L */
    {0x7F,0x40,0x40,0x40,0x40},

    /* M */
    {0x7F,0x02,0x0C,0x02,0x7F},

    /* N */
    {0x7F,0x04,0x08,0x10,0x7F},

    /* O */
    {0x3E,0x41,0x41,0x41,0x3E},

    /* P */
    {0x7F,0x09,0x09,0x09,0x06},

    /* Q */
    {0x3E,0x41,0x51,0x21,0x5E},

    /* R */
    {0x7F,0x09,0x19,0x29,0x46},

    /* S */
    {0x46,0x49,0x49,0x49,0x31},

    /* T */
    {0x01,0x01,0x7F,0x01,0x01},

    /* U */
    {0x3F,0x40,0x40,0x40,0x3F},

    /* V */
    {0x1F,0x20,0x40,0x20,0x1F},

    /* W */
    {0x3F,0x40,0x38,0x40,0x3F},

    /* X */
    {0x63,0x14,0x08,0x14,0x63},

    /* Y */
    {0x07,0x08,0x70,0x08,0x07},

    /* Z */
    {0x61,0x51,0x49,0x45,0x43}
};


/* ============================================================
 * I2C Write
 * ============================================================ */

static int oled_i2c_write(uint8_t *data, size_t length)
{
    struct {
        i2c_send_t hdr;
        uint8_t data[256];
    } msg;

    if (length > sizeof(msg.data))
        return -1;

    msg.hdr.slave.addr = OLED_I2C_ADDR;
    msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.hdr.len = length;
    msg.hdr.stop = 1;

    memcpy(msg.data, data, length);

    return devctl(oled_fd,
                  DCMD_I2C_SEND,
                  &msg,
                  sizeof(msg.hdr) + length,
                  NULL);
}


/* ============================================================
 * Send OLED command
 * ============================================================ */

static int oled_command(uint8_t command)
{
    uint8_t data[2];
    int rc;

    data[0] = 0x00;       /* Command control byte */
    data[1] = command;

    rc = oled_i2c_write(data, 2);
    if (rc != 0)
    {
        fprintf(stderr, "OLED: command 0x%02X failed, devctl rc=%d\n",
                command, rc);
    }
    return rc;
}


/* ============================================================
 * Send OLED data
 * ============================================================ */

static int oled_data(uint8_t *data, size_t length)
{
    uint8_t buffer[256];
    int rc;

    if (length > 255)
        return -1;

    buffer[0] = 0x40;     /* Data control byte */

    memcpy(&buffer[1], data, length);

    rc = oled_i2c_write(buffer, length + 1);
    if (rc != 0)
    {
        fprintf(stderr, "OLED: data write failed, devctl rc=%d\n", rc);
    }
    return rc;
}


/* ============================================================
 * Set page
 * ============================================================ */

static void oled_set_page(uint8_t page)
{
    oled_command(0xB0 | page);
}


/* ============================================================
 * Set column
 * ============================================================ */

static void oled_set_column(uint8_t column)
{
    oled_command(0x00 | (column & 0x0F));
    oled_command(0x10 | ((column >> 4) & 0x0F));
}


/* ============================================================
 * Initialize SH1106
 * ============================================================ */

int oled_init(const char *i2c_device)
{
    oled_fd = open(i2c_device, O_RDWR);

    if (oled_fd == -1)
    {
        perror("OLED: Unable to open I2C");
        return -1;
    }

    usleep(100000);

    /* SH1106 initialization */

    oled_command(0xAE);       /* Display OFF */

    oled_command(0xD5);       /* Clock */
    oled_command(0x80);

    oled_command(0xA8);       /* Multiplex */
    oled_command(0x3F);

    oled_command(0xD3);       /* Display offset */
    oled_command(0x00);

    oled_command(0x40);       /* Start line */

    oled_command(0xAD);       /* DC-DC control */
    oled_command(0x8B);

    oled_command(0xA1);       /* Segment remap */

    oled_command(0xC8);       /* COM scan direction */

    oled_command(0xDA);       /* COM pins */
    oled_command(0x12);

    oled_command(0x81);       /* Contrast */
    oled_command(0x7F);

    oled_command(0xD9);       /* Pre-charge */
    oled_command(0x22);

    oled_command(0xDB);       /* VCOM */
    oled_command(0x35);

    oled_command(0xA4);       /* Display RAM */

    oled_command(0xA6);       /* Normal display */

    oled_command(0xAF);       /* Display ON */

    oled_clear();

    return 0;
}


/* ============================================================
 * Clear buffer
 * ============================================================ */

void oled_clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));

    oled_update();

    cursor_x = 0;
    cursor_page = 0;
}


/* ============================================================
 * Update entire display
 * ============================================================ */

void oled_update(void)
{
    uint8_t page;

    for (page = 0; page < 8; page++)
    {
        oled_set_page(page);

        /* SH1106 has 2-column offset */
        oled_set_column(2);

        oled_data(&oled_buffer[page * OLED_WIDTH],
                  OLED_WIDTH);
    }
}


/* ============================================================
 * Set cursor
 * ============================================================ */

void oled_set_cursor(uint8_t x, uint8_t page)
{
    if (x >= OLED_WIDTH)
        x = 0;

    if (page >= 8)
        page = 0;

    cursor_x = x;
    cursor_page = page;
}


/* ============================================================
 * Draw one character
 * ============================================================ */

static void oled_draw_char(char c)
{
    int index;
    int i;

    if (c >= ' ' && c <= 'Z')
    {
        index = c - ' ';
    }
    else
    {
        index = 0;
    }

    if (cursor_x > OLED_WIDTH - 6)
    {
        cursor_x = 0;
        cursor_page++;

        if (cursor_page >= 8)
            cursor_page = 0;
    }

    for (i = 0; i < 5; i++)
    {
        oled_buffer[cursor_page * OLED_WIDTH +
                    cursor_x + i] =
                    font5x7[index][i];
    }

    /* Space between characters */
    oled_buffer[cursor_page * OLED_WIDTH +
                cursor_x + 5] = 0x00;

    cursor_x += 6;
}


/* ============================================================
 * Print string
 * ============================================================ */

void oled_print(const char *text)
{
    while (*text)
    {
        if (*text == '\n')
        {
            cursor_x = 0;
            cursor_page++;

            if (cursor_page >= 8)
                cursor_page = 0;
        }
        else
        {
            oled_draw_char(*text);
        }

        text++;
    }

    oled_update();
}


/* ============================================================
 * Print integer
 * ============================================================ */

void oled_print_int(int value)
{
    char buffer[20];

    snprintf(buffer, sizeof(buffer), "%d", value);

    oled_print(buffer);
}


/* ============================================================
 * Print float
 * ============================================================ */

void oled_print_float(float value, int decimals)
{
    char buffer[32];

    snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);

    oled_print(buffer);
}


/* ============================================================
 * Display ON
 * ============================================================ */

void oled_display_on(void)
{
    oled_command(0xAF);
}


/* ============================================================
 * Display OFF
 * ============================================================ */

void oled_display_off(void)
{
    oled_command(0xAE);
}


/* ============================================================
 * Close OLED
 * ============================================================ */

void oled_close(void)
{
    if (oled_fd >= 0)
    {
        oled_display_off();

        close(oled_fd);

        oled_fd = -1;
    }
}
