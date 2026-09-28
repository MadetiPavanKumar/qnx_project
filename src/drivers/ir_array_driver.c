#include "ir_array_driver.h"

#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <rpi_gpio.h>


/* ============================================================
 * INTERNAL DRIVER STATE
 * ============================================================ */

typedef struct
{
    pthread_mutex_t mutex;

    uint8_t initialized;
    uint8_t healthy;

    IR_Config config;

    uint32_t sequence;
    uint32_t read_failures;

} IR_DriverState;


static IR_DriverState g_ir =
{
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .initialized = 0,
    .healthy = 0,
    .sequence = 0,
    .read_failures = 0
};


/* ============================================================
 * CONFIGURATION VALIDATION
 * ============================================================ */

static int validate_config(const IR_Config *config)
{
    int i;

    if (config == NULL)
    {
        return IR_INVALID_ARGUMENT;
    }

    for (i = 0; i < IR_CHANNEL_COUNT; i++)
    {
        /*
         * Raspberry Pi GPIO numbers supported by your API:
         * GPIO0 ... GPIO27
         */
        if (config->gpio[i] < GPIO0 ||
            config->gpio[i] > GPIO27)
        {
            return IR_INVALID_ARGUMENT;
        }
    }

    if (config->active_low > 1)
    {
        return IR_INVALID_ARGUMENT;
    }

    return IR_SUCCESS;
}


/* ============================================================
 * CONFIGURE ONE GPIO AS INPUT
 * ============================================================ */

static int configure_gpio(int gpio)
{
    int ret;

    ret = rpi_gpio_setup(gpio, GPIO_IN);

    if (ret != GPIO_SUCCESS)
    {
        return IR_GPIO_CONFIG_FAILED;
    }

    /*
     * TCRT5000L module outputs are actively driven,
     * so normally no internal pull resistor is required.
     *
     * Therefore we intentionally don't enable pull-up/down.
     */

    return IR_SUCCESS;
}


/* ============================================================
 * READ ONE GPIO
 * ============================================================ */

static int read_gpio(int gpio, uint8_t *value)
{
    unsigned level;
    int ret;

    if (value == NULL)
    {
        return IR_INVALID_ARGUMENT;
    }

    level = GPIO_LOW;

    ret = rpi_gpio_input(gpio, &level);

    if (ret != GPIO_SUCCESS)
    {
        return IR_GPIO_READ_FAILED;
    }

    if (level == GPIO_HIGH)
    {
        *value = 1U;
    }
    else if (level == GPIO_LOW)
    {
        *value = 0U;
    }
    else
    {
        /*
         * Unexpected GPIO level.
         *
         * Do NOT guess the sensor state.
         */
        return IR_GPIO_READ_FAILED;
    }

    return IR_SUCCESS;
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

int IRArray_Init(const IR_Config *config)
{
    int i;
    int ret;

    ret = validate_config(config);

    if (ret != IR_SUCCESS)
    {
        return ret;
    }

    pthread_mutex_lock(&g_ir.mutex);

    /*
     * Already initialized.
     *
     * Treat initialization as idempotent.
     */
    if (g_ir.initialized)
    {
        pthread_mutex_unlock(&g_ir.mutex);
        return IR_SUCCESS;
    }

    /*
     * Copy configuration.
     */
    memcpy(&g_ir.config, config, sizeof(IR_Config));

    /*
     * Configure all five GPIOs.
     */
    for (i = 0; i < IR_CHANNEL_COUNT; i++)
    {
        ret = configure_gpio(g_ir.config.gpio[i]);

        if (ret != IR_SUCCESS)
        {
            /*
             * Fail-safe initialization.
             *
             * Driver is NOT considered operational.
             */
            g_ir.initialized = 0;
            g_ir.healthy = 0;

            pthread_mutex_unlock(&g_ir.mutex);

            return ret;
        }
    }

    g_ir.sequence = 0;
    g_ir.read_failures = 0;

    g_ir.initialized = 1;
    g_ir.healthy = 1;

    pthread_mutex_unlock(&g_ir.mutex);

    return IR_SUCCESS;
}


/* ============================================================
 * READ ALL FIVE IR CHANNELS
 * ============================================================ */

int IRArray_Read(IR_Reading *reading)
{
    int i;
    int ret;

    uint8_t raw_value;

    if (reading == NULL)
    {
        return IR_INVALID_ARGUMENT;
    }

    /*
     * Start with an invalid/empty structure.
     *
     * This is important for fail-safe operation.
     */
    memset(reading, 0, sizeof(IR_Reading));

    reading->valid = 0;
    reading->position = IR_POSITION_UNKNOWN;

    pthread_mutex_lock(&g_ir.mutex);

    if (!g_ir.initialized)
    {
        pthread_mutex_unlock(&g_ir.mutex);

        return IR_NOT_INITIALIZED;
    }

    /*
     * Read every channel.
     *
     * If even ONE GPIO read fails, the COMPLETE sample
     * is rejected.
     */
    for (i = 0; i < IR_CHANNEL_COUNT; i++)
    {
        ret = read_gpio(
            g_ir.config.gpio[i],
            &raw_value
        );

        if (ret != IR_SUCCESS)
        {
            g_ir.read_failures++;

            /*
             * Mark driver unhealthy after repeated failures.
             */
            if (g_ir.read_failures >= IR_MAX_READ_FAILURES)
            {
                g_ir.healthy = 0;
            }

            reading->valid = 0;
            reading->line_detected = 0;
            reading->position = IR_POSITION_UNKNOWN;
            reading->read_failures = g_ir.read_failures;

            pthread_mutex_unlock(&g_ir.mutex);

            return IR_GPIO_READ_FAILED;
        }

        /*
         * Convert electrical level into logical sensor state.
         *
         * active_low = 0:
         *
         * HIGH -> detected
         * LOW  -> not detected
         *
         *
         * active_low = 1:
         *
         * LOW  -> detected
         * HIGH -> not detected
         */
        if (g_ir.config.active_low)
        {
            reading->sensor[i] =
                (raw_value == 0U) ? 1U : 0U;
        }
        else
        {
            reading->sensor[i] =
                (raw_value == 1U) ? 1U : 0U;
        }
    }

    /*
     * Entire 5-channel read succeeded.
     */
    g_ir.read_failures = 0;
    g_ir.healthy = 1;

    g_ir.sequence++;

    reading->sequence = g_ir.sequence;
    reading->read_failures = 0;

    reading->valid = 1;

    /*
     * Determine whether any sensor detects the line.
     */
    reading->line_detected = 0;

    for (i = 0; i < IR_CHANNEL_COUNT; i++)
    {
        if (reading->sensor[i])
        {
            reading->line_detected = 1;
            break;
        }
    }

    /*
     * Determine position.
     */
    reading->position =
        IRArray_GetPosition(reading);

    pthread_mutex_unlock(&g_ir.mutex);

    return IR_SUCCESS;
}


/* ============================================================
 * DETERMINE LINE POSITION
 * ============================================================ */

IR_Position IRArray_GetPosition(const IR_Reading *reading)
{
    uint8_t s1;
    uint8_t s2;
    uint8_t s3;
    uint8_t s4;
    uint8_t s5;

    if (reading == NULL)
    {
        return IR_POSITION_UNKNOWN;
    }

    if (!reading->valid)
    {
        return IR_POSITION_UNKNOWN;
    }

    s1 = reading->sensor[0];
    s2 = reading->sensor[1];
    s3 = reading->sensor[2];
    s4 = reading->sensor[3];
    s5 = reading->sensor[4];


    /*
     * ========================================================
     * NO LINE
     * ========================================================
     */

    if ((s1 == 0) &&
        (s2 == 0) &&
        (s3 == 0) &&
        (s4 == 0) &&
        (s5 == 0))
    {
        return IR_POSITION_LOST;
    }


    /*
     * ========================================================
     * FAR LEFT
     * ========================================================
     */

    if ((s1 == 1) &&
        (s2 == 0) &&
        (s3 == 0) &&
        (s4 == 0) &&
        (s5 == 0))
    {
        return IR_POSITION_FAR_LEFT;
    }


    /*
     * ========================================================
     * LEFT
     * ========================================================
     */

    if ((s2 == 1) &&
        (s3 == 0))
    {
        return IR_POSITION_LEFT;
    }


    /*
     * ========================================================
     * SLIGHT LEFT
     * ========================================================
     */

    if ((s2 == 1) &&
        (s3 == 1))
    {
        return IR_POSITION_SLIGHT_LEFT;
    }


    /*
     * ========================================================
     * CENTER
     * ========================================================
     */

    if ((s3 == 1) &&
        (s1 == 0) &&
        (s5 == 0))
    {
        return IR_POSITION_CENTER;
    }


    /*
     * ========================================================
     * SLIGHT RIGHT
     * ========================================================
     */

    if ((s3 == 1) &&
        (s4 == 1))
    {
        return IR_POSITION_SLIGHT_RIGHT;
    }


    /*
     * ========================================================
     * RIGHT
     * ========================================================
     */

    if ((s4 == 1) &&
        (s3 == 0))
    {
        return IR_POSITION_RIGHT;
    }


    /*
     * ========================================================
     * FAR RIGHT
     * ========================================================
     */

    if ((s5 == 1) &&
        (s4 == 0) &&
        (s3 == 0))
    {
        return IR_POSITION_FAR_RIGHT;
    }


    /*
     * ========================================================
     * UNKNOWN PATTERN
     * ========================================================
     *
     * Do NOT guess.
     *
     * Navigation should treat this as an abnormal condition.
     */

    return IR_POSITION_UNKNOWN;
}


/* ============================================================
 * POSITION STRING
 * ============================================================ */

const char *IRArray_PositionString(IR_Position position)
{
    switch (position)
    {
        case IR_POSITION_FAR_LEFT:
            return "FAR_LEFT";

        case IR_POSITION_LEFT:
            return "LEFT";

        case IR_POSITION_SLIGHT_LEFT:
            return "SLIGHT_LEFT";

        case IR_POSITION_CENTER:
            return "CENTER";

        case IR_POSITION_SLIGHT_RIGHT:
            return "SLIGHT_RIGHT";

        case IR_POSITION_RIGHT:
            return "RIGHT";

        case IR_POSITION_FAR_RIGHT:
            return "FAR_RIGHT";

        case IR_POSITION_LOST:
            return "LINE_LOST";

        case IR_POSITION_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * DRIVER HEALTH
 * ============================================================ */

int IRArray_IsHealthy(void)
{
    int healthy;

    pthread_mutex_lock(&g_ir.mutex);

    healthy =
        (g_ir.initialized && g_ir.healthy);

    pthread_mutex_unlock(&g_ir.mutex);

    return healthy;
}


/* ============================================================
 * CLOSE DRIVER
 * ============================================================ */

void IRArray_Close(void)
{
    pthread_mutex_lock(&g_ir.mutex);

    /*
     * We don't call rpi_gpio_cleanup() here because the GPIO
     * resource manager is shared by other drivers in your
     * application.
     *
     * Calling global cleanup from this driver could break:
     *
     *   L298N
     *   Encoder
     *   HC-SR04
     *   other GPIO users
     *
     * The application should perform global GPIO cleanup
     * when the entire application exits.
     */

    g_ir.initialized = 0;
    g_ir.healthy = 0;

    g_ir.sequence = 0;
    g_ir.read_failures = 0;

    memset(
        &g_ir.config,
        0,
        sizeof(IR_Config)
    );

    pthread_mutex_unlock(&g_ir.mutex);
}
