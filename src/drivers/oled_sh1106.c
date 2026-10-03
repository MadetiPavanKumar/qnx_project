/*
 * oled_sh1106.c
 * ---------------------------------------------------------------
 * QNX I2C driver for a 128x64 SH1106 OLED (I2C address 0x3C).
 *
 * QNX concept: on QNX the I2C bus is a RESOURCE MANAGER, i.e. a device
 * file such as /dev/i2c1. We open() it like a file and send bytes to
 * the OLED with devctl(DCMD_I2C_SEND). The message for devctl is
 *      [ i2c_send_t header | payload bytes ]
 * where the header holds the slave address and the payload length.
 *
 * SH1106 protocol in one paragraph: every I2C transfer starts with a
 * CONTROL byte: 0x00 = "all following bytes are commands",
 * 0x40 = "all following bytes are pixel data". Pixel data is written
 * page by page: pick a page (0xB0+page) and a start column (two
 * commands: low nibble 0x0n, high nibble 0x1n), then send the bytes.
 * One byte = 8 vertical pixels of one column, bit 0 = top pixel.
 *
 * NOT hardware tested by the author - if your picture is upside down
 * or shifted, use OLED_ROTATE_180 / OLED_COLUMN_OFFSET in the header.
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>
#include "oled_sh1106.h"

static int     oled_fd = -1;
static uint8_t framebuffer[OLED_WIDTH * OLED_PAGES];   /* the RAM copy  */

/* ============================================================
 * 5x7 font - one byte per column, bit 0 = top pixel.
 * Only what this project needs: digits, A-Z and a few symbols.
 * ============================================================ */
typedef struct { char ch; uint8_t col[5]; } glyph_t;

static const glyph_t font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'%', {0x23, 0x13, 0x08, 0x64, 0x62}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

/* find_glyph: look a character up in the font table (lower case is
 * folded to upper case). Returns NULL for characters we don't have;
 * the callers then leave a blank gap. */
static const uint8_t *find_glyph(char c)
{
    size_t i;
    c = (char)toupper((unsigned char)c);
    for (i = 0; i < sizeof(font) / sizeof(font[0]); i++)
        if (font[i].ch == c)
            return font[i].col;
    return NULL;
}

/* ============================================================
 * Low-level I2C
 * ============================================================ */

/* i2c_write: send `len` bytes to the OLED in ONE I2C transfer.
 * Builds the [header | payload] message devctl expects. Returns 0 on
 * success, -1 on error. */
static int i2c_write(const uint8_t *data, size_t len)
{
    struct {
        i2c_send_t hdr;
        uint8_t    payload[OLED_WIDTH + 8];
    } msg;

    if (oled_fd < 0 || len > sizeof(msg.payload))
        return -1;

    memset(&msg.hdr, 0, sizeof(msg.hdr));
    msg.hdr.slave.addr = OLED_I2C_ADDR;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.len        = (uint32_t)len;
    msg.hdr.stop       = 1;
    memcpy(msg.payload, data, len);

    return (devctl(oled_fd, DCMD_I2C_SEND, &msg, sizeof(msg.hdr) + len, NULL) == 0) ? 0 : -1;
}

/* send_commands: send n command bytes in one transfer (control byte
 * 0x00 first, meaning "everything after me is a command"). */
static int send_commands(const uint8_t *cmds, size_t n)
{
    uint8_t buf[32];

    if (n + 1 > sizeof(buf))
        return -1;
    buf[0] = 0x00;
    memcpy(&buf[1], cmds, n);
    return i2c_write(buf, n + 1);
}

/* send_data: send n pixel bytes in one transfer (control byte 0x40). */
static int send_data(const uint8_t *pixels, size_t n)
{
    uint8_t buf[OLED_WIDTH + 1];

    if (n + 1 > sizeof(buf))
        return -1;
    buf[0] = 0x40;
    memcpy(&buf[1], pixels, n);
    return i2c_write(buf, n + 1);
}

/* ============================================================
 * Public API
 * ============================================================ */

/* oled_init: open the I2C device and run the SH1106 start-up sequence
 * (clock, multiplex ratio, charge pump, orientation, contrast...),
 * then show a blank screen. */
int oled_init(const char *i2c_device)
{
    static const uint8_t init_seq[] = {
        0xAE,             /* display OFF while configuring              */
        0xD5, 0x80,       /* clock divide / oscillator frequency        */
        0xA8, 0x3F,       /* multiplex ratio: 64 rows                   */
        0xD3, 0x00,       /* display offset: none                       */
        0x40,             /* start line = 0                             */
        0xAD, 0x8B,       /* internal DC-DC converter ON                */
#if OLED_ROTATE_180
        0xA0, 0xC0,       /* segment remap + COM scan: rotated 180      */
#else
        0xA1, 0xC8,       /* segment remap + COM scan: normal           */
#endif
        0xDA, 0x12,       /* COM pin hardware configuration             */
        0x81, 0xCF,       /* contrast                                   */
        0xD9, 0xF1,       /* pre-charge period                          */
        0xDB, 0x40,       /* VCOM deselect level                        */
        0xA4,             /* show RAM content                           */
        0xA6,             /* normal (not inverted) display              */
        0xAF              /* display ON                                 */
    };

    oled_fd = open(i2c_device, O_RDWR);
    if (oled_fd < 0) {
        perror("oled_init: open");
        return -1;
    }
    if (send_commands(init_seq, sizeof(init_seq)) != 0) {
        fprintf(stderr, "oled_init: no answer from OLED at 0x%02X on %s\n",
                OLED_I2C_ADDR, i2c_device);
        close(oled_fd);
        oled_fd = -1;
        return -1;
    }
    oled_clear();
    return oled_flush();
}

/* oled_close: display off, release the device. */
void oled_close(void)
{
    static const uint8_t off = 0xAE;

    if (oled_fd >= 0) {
        send_commands(&off, 1);
        close(oled_fd);
        oled_fd = -1;
    }
}

/* oled_clear: black framebuffer (RAM only). */
void oled_clear(void)
{
    memset(framebuffer, 0, sizeof(framebuffer));
}

/* oled_print: copy the 5 font columns of every character into the
 * framebuffer, 6 pixels per character (5 + 1 blank column). Text that
 * would run past the right edge is cut off. */
void oled_print(int x, int page, const char *text)
{
    if (page < 0 || page >= OLED_PAGES)
        return;

    for (; *text != '\0' && x >= 0 && x + 5 <= OLED_WIDTH; text++, x += 6) {
        const uint8_t *g = find_glyph(*text);
        int c;
        for (c = 0; c < 5; c++)
            framebuffer[page * OLED_WIDTH + x + c] = g ? g[c] : 0x00;
    }
}

/* oled_print_big: double-size text, centred.
 *   - every glyph column is written twice            -> 2x wider
 *   - every pixel (bit) becomes two stacked bits     -> 2x taller
 * The 7 glyph rows become 14 rows = 16 bits: the low byte goes into
 * `page`, the high byte into `page + 1`. Each character takes 12
 * pixels (10 for the letter + 2 blank). */
void oled_print_big(int page, const char *text)
{
    int len = (int)strlen(text);
    int x   = (OLED_WIDTH - (len * 12 - 2)) / 2;
    int i, c, row;

    if (page < 0 || page + 1 >= OLED_PAGES)
        return;
    if (x < 0)
        x = 0;

    for (i = 0; i < len; i++, x += 12) {
        const uint8_t *g = find_glyph(text[i]);

        if (g == NULL || x + 10 > OLED_WIDTH)
            continue;

        for (c = 0; c < 5; c++) {
            unsigned stretched = 0;
            for (row = 0; row < 7; row++)
                if (g[c] & (1u << row))
                    stretched |= 3u << (2 * row);      /* 1 pixel -> 2 */

            framebuffer[page * OLED_WIDTH + x + 2 * c]           = stretched & 0xFF;
            framebuffer[page * OLED_WIDTH + x + 2 * c + 1]       = stretched & 0xFF;
            framebuffer[(page + 1) * OLED_WIDTH + x + 2 * c]     = (stretched >> 8) & 0xFF;
            framebuffer[(page + 1) * OLED_WIDTH + x + 2 * c + 1] = (stretched >> 8) & 0xFF;
        }
    }
}

/* oled_flush: write the framebuffer to the panel, one page at a time:
 *   command: select page            0xB0 | page
 *   command: start column (2 bytes) 0x00 | low nibble, 0x10 | high nibble
 *   data:    128 pixel bytes
 * Returns 0 if every transfer worked, otherwise -1. */
int oled_flush(void)
{
    int page, rc = 0;

    for (page = 0; page < OLED_PAGES; page++) {
        uint8_t cmds[3];
        cmds[0] = (uint8_t)(0xB0 | page);
        cmds[1] = (uint8_t)(0x00 | (OLED_COLUMN_OFFSET & 0x0F));
        cmds[2] = (uint8_t)(0x10 | ((OLED_COLUMN_OFFSET >> 4) & 0x0F));

        if (send_commands(cmds, sizeof(cmds)) != 0 ||
            send_data(&framebuffer[page * OLED_WIDTH], OLED_WIDTH) != 0)
            rc = -1;
    }
    return rc;
}
