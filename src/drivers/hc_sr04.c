/*
 * ============================================================
 * HC-SR04 QNX DRIVER
 * ============================================================
 *
 * Raspberry Pi 4 / BCM2711
 *
 * TRIG:
 *      BCM GPIO5
 *      Physical Pin 29
 *
 * ECHO:
 *      BCM GPIO6
 *      Physical Pin 31
 *
 * IMPORTANT:
 *
 * HC-SR04 ECHO can be approximately 5 V.
 *
 * DO NOT connect ECHO directly to Raspberry Pi GPIO.
 *
 * Recommended divider:
 *
 * HC-SR04 ECHO
 *       |
 *      1K
 *       |
 *       +---------- GPIO6
 *       |
 *      2K
 *       |
 *      GND
 *
 * ============================================================
 */

#include "hc_sr04.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sched.h>

#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>


/* ============================================================
 * BCM2711 GPIO
 * ============================================================ */

#define GPIO_BASE_ADDRESS      0xFE200000UL
#define GPIO_MAP_SIZE          0x1000


/*
 * GPIO register offsets.
 */

#define GPFSEL0                0x00

#define GPSET0                 0x1C

#define GPCLR0                 0x28

#define GPLEV0                 0x34

#define GPPUPPDN0              0xE4


/* ============================================================
 * GPIO FUNCTION
 * ============================================================ */

#define GPIO_INPUT             0

#define GPIO_OUTPUT            1


/* ============================================================
 * INTERNAL HELPERS
 * ============================================================ */


/*
 * Return current ClockCycles() value.
 */
static inline uint64_t hc_cycles(void)
{
    return ClockCycles();
}


/*
 * Convert ClockCycles() to microseconds.
 *
 * We use integer arithmetic to avoid floating point
 * calculations inside the critical timing loops.
 */
static inline uint64_t cycles_to_us(
        uint64_t cycles,
        uint64_t cycles_per_sec)
{
    if (cycles_per_sec == 0)
    {
        return 0;
    }

    return (cycles * 1000000ULL) / cycles_per_sec;
}


/*
 * Delay using ClockCycles().
 *
 * This is used for the 10 us HC-SR04 trigger pulse.
 *
 * Unlike usleep(), this avoids scheduler-dependent
 * microsecond delays.
 */
static void delay_us(
        uint64_t us,
        uint64_t cycles_per_sec)
{
    uint64_t start;
    uint64_t required_cycles;

    if (us == 0)
    {
        return;
    }

    start = hc_cycles();

    required_cycles =
        (cycles_per_sec * us) / 1000000ULL;

    while ((hc_cycles() - start) < required_cycles)
    {
        /*
         * Deliberately busy-wait.
         *
         * This is only used for very short timing intervals.
         */
    }
}


/*
 * Return pointer to a GPIO register.
 */
static inline volatile uint32_t *
gpio_reg(
        hc_sr04_t *sensor,
        uint32_t offset)
{
    return (volatile uint32_t *)
        ((uintptr_t)sensor->gpio_base + offset);
}


/*
 * Configure GPIO as input or output.
 */
static void gpio_set_function(
        hc_sr04_t *sensor,
        unsigned gpio,
        unsigned function)
{
    unsigned reg_index;
    unsigned shift;
    uint32_t value;

    reg_index = gpio / 10;
    shift = (gpio % 10) * 3;

    value = gpio_reg(
                sensor,
                GPFSEL0 + (reg_index * 4))[0];

    value &= ~(7U << shift);

    value |=
        ((function & 7U) << shift);

    gpio_reg(
        sensor,
        GPFSEL0 + (reg_index * 4))[0] = value;
}


/*
 * Configure GPIO with no pull-up/down.
 *
 * BCM2711:
 *
 * Each GPIO has 2 bits in GPPUPPDN registers.
 */
static void gpio_disable_pull(
        hc_sr04_t *sensor,
        unsigned gpio)
{
    unsigned reg_index;
    unsigned shift;
    uint32_t value;

    reg_index = gpio / 16;
    shift = (gpio % 16) * 2;

    value = gpio_reg(
                sensor,
                GPPUPPDN0 + (reg_index * 4))[0];

    value &= ~(3U << shift);

    /*
     * 00 = no pull
     */
    gpio_reg(
        sensor,
        GPPUPPDN0 + (reg_index * 4))[0] = value;
}


/*
 * Set GPIO HIGH.
 */
static inline void gpio_high(
        hc_sr04_t *sensor,
        unsigned gpio)
{
    gpio_reg(sensor, GPSET0)[0] =
        (1U << gpio);
}


/*
 * Set GPIO LOW.
 */
static inline void gpio_low(
        hc_sr04_t *sensor,
        unsigned gpio)
{
    gpio_reg(sensor, GPCLR0)[0] =
        (1U << gpio);
}


/*
 * Read GPIO level.
 */
static inline int gpio_read(
        hc_sr04_t *sensor,
        unsigned gpio)
{
    uint32_t value;

    value = gpio_reg(sensor, GPLEV0)[0];

    return
        (value & (1U << gpio)) ? 1 : 0;
}


/* ============================================================
 * CONFIGURATION VALIDATION
 * ============================================================ */

static bool config_valid(
        const hc_sr04_config_t *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    /*
     * BCM2711 GPIO numbers.
     */
    if (cfg->trig_gpio > 27)
    {
        return false;
    }

    if (cfg->echo_gpio > 27)
    {
        return false;
    }

    /*
     * Trigger and ECHO must be different.
     */
    if (cfg->trig_gpio == cfg->echo_gpio)
    {
        return false;
    }

    if (cfg->min_distance_cm <= 0.0f)
    {
        return false;
    }

    if (cfg->max_distance_cm <=
        cfg->min_distance_cm)
    {
        return false;
    }

    if (cfg->speed_of_sound_cm_per_us <= 0.0f)
    {
        return false;
    }

    if (cfg->measurement_interval_us < 1000ULL)
    {
        return false;
    }

    if (cfg->rising_timeout_us == 0)
    {
        return false;
    }

    if (cfg->falling_timeout_us == 0)
    {
        return false;
    }

    if (cfg->min_pulse_us >=
        cfg->max_pulse_us)
    {
        return false;
    }

    return true;
}


/* ============================================================
 * ECHO RECOVERY
 * ============================================================ */


/*
 * Wait until ECHO is LOW.
 *
 * This function is extremely important.
 *
 * If an earlier measurement leaves ECHO HIGH,
 * we must NOT send another trigger and accidentally
 * pair the new measurement with the old ECHO state.
 */
static hc_sr04_status_t wait_echo_low(
        hc_sr04_t *sensor)
{
    uint64_t start;
    uint64_t elapsed;

    start = hc_cycles();

    while (gpio_read(
                sensor,
                sensor->cfg.echo_gpio))
    {
        elapsed =
            cycles_to_us(
                hc_cycles() - start,
                sensor->cycles_per_sec);

        if (elapsed >=
            HC_SR04_ECHO_RECOVERY_TIMEOUT_US)
        {
            return HC_SR04_ECHO_TIMEOUT;
        }
    }

    return HC_SR04_OK;
}


/* ============================================================
 * TRIGGER
 * ============================================================ */

static void send_trigger(
        hc_sr04_t *sensor)
{
    /*
     * Guarantee LOW before trigger.
     */
    gpio_low(
        sensor,
        sensor->cfg.trig_gpio);

    delay_us(
        2,
        sensor->cycles_per_sec);


    /*
     * HC-SR04 requires >= 10 us HIGH.
     */
    gpio_high(
        sensor,
        sensor->cfg.trig_gpio);

    delay_us(
        10,
        sensor->cycles_per_sec);


    /*
     * End trigger.
     */
    gpio_low(
        sensor,
        sensor->cfg.trig_gpio);
}


/* ============================================================
 * WAIT FOR RISING EDGE
 * ============================================================ */

static hc_sr04_status_t
wait_for_echo_rising(
        hc_sr04_t *sensor,
        uint64_t *rising_cycles)
{
    uint64_t start;
    uint64_t elapsed;

    start = hc_cycles();

    while (!gpio_read(
                sensor,
                sensor->cfg.echo_gpio))
    {
        elapsed =
            cycles_to_us(
                hc_cycles() - start,
                sensor->cycles_per_sec);

        if (elapsed >=
            sensor->cfg.rising_timeout_us)
        {
            return HC_SR04_NO_ECHO;
        }
    }


    /*
     * IMPORTANT:
     *
     * Capture the timestamp immediately after detecting
     * HIGH.
     *
     * Do not use clock_gettime() after a QNX event handler.
     */
    *rising_cycles = hc_cycles();

    return HC_SR04_OK;
}


/* ============================================================
 * WAIT FOR FALLING EDGE
 * ============================================================ */

static hc_sr04_status_t
wait_for_echo_falling(
        hc_sr04_t *sensor,
        uint64_t rising_cycles,
        uint64_t *falling_cycles)
{
    uint64_t elapsed;
    uint64_t pulse_us;

    while (gpio_read(
                sensor,
                sensor->cfg.echo_gpio))
    {
        elapsed =
            cycles_to_us(
                hc_cycles() - rising_cycles,
                sensor->cycles_per_sec);

        /*
         * Do not allow ECHO HIGH to remain indefinitely.
         */
        if (elapsed >=
            sensor->cfg.falling_timeout_us)
        {
            /*
             * Attempt recovery.
             */
            uint64_t recovery_start =
                hc_cycles();

            while (gpio_read(
                        sensor,
                        sensor->cfg.echo_gpio))
            {
                uint64_t recovery_elapsed =
                    cycles_to_us(
                        hc_cycles() - recovery_start,
                        sensor->cycles_per_sec);

                if (recovery_elapsed >=
                    HC_SR04_ECHO_RECOVERY_TIMEOUT_US)
                {
                    break;
                }
            }

            return HC_SR04_ECHO_TIMEOUT;
        }
    }


    /*
     * Capture falling timestamp immediately.
     */
    *falling_cycles = hc_cycles();


    pulse_us =
        cycles_to_us(
            *falling_cycles - rising_cycles,
            sensor->cycles_per_sec);


    /*
     * Physical validation.
     */
    if (pulse_us <
        sensor->cfg.min_pulse_us)
    {
        return HC_SR04_TOO_CLOSE;
    }


    if (pulse_us >
        sensor->cfg.max_pulse_us)
    {
        return HC_SR04_OUT_OF_RANGE;
    }


    return HC_SR04_OK;
}


/* ============================================================
 * STATISTICS
 * ============================================================ */

static void stats_record_success(
        hc_sr04_t *sensor,
        float distance_cm,
        uint64_t echo_us)
{
    sensor->stats.total_measurements++;

    sensor->stats.successful_measurements++;

    sensor->stats.last_distance_cm =
        distance_cm;

    sensor->stats.last_echo_us =
        echo_us;


    if (sensor->stats.successful_measurements == 1)
    {
        sensor->stats.minimum_distance_cm =
            distance_cm;

        sensor->stats.maximum_distance_cm =
            distance_cm;
    }
    else
    {
        if (distance_cm <
            sensor->stats.minimum_distance_cm)
        {
            sensor->stats.minimum_distance_cm =
                distance_cm;
        }

        if (distance_cm >
            sensor->stats.maximum_distance_cm)
        {
            sensor->stats.maximum_distance_cm =
                distance_cm;
        }
    }
}


/* ============================================================
 * DEFAULT CONFIGURATION
 * ============================================================ */

void hc_sr04_config_default(
        hc_sr04_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(
        config,
        0,
        sizeof(*config));


    config->trig_gpio =
        HC_SR04_DEFAULT_TRIG_GPIO;

    config->echo_gpio =
        HC_SR04_DEFAULT_ECHO_GPIO;


    config->min_distance_cm =
        HC_SR04_DEFAULT_MIN_DISTANCE_CM;

    config->max_distance_cm =
        HC_SR04_DEFAULT_MAX_DISTANCE_CM;


    config->speed_of_sound_cm_per_us =
        HC_SR04_DEFAULT_SPEED_OF_SOUND;


    config->measurement_interval_us =
        HC_SR04_DEFAULT_MEASUREMENT_INTERVAL_US;


    config->rising_timeout_us =
        HC_SR04_DEFAULT_RISING_TIMEOUT_US;


    config->falling_timeout_us =
        HC_SR04_DEFAULT_FALLING_TIMEOUT_US;


    config->min_pulse_us =
        HC_SR04_DEFAULT_MIN_PULSE_US;


    config->max_pulse_us =
        HC_SR04_DEFAULT_MAX_PULSE_US;
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

hc_sr04_status_t hc_sr04_init(
        hc_sr04_t *sensor,
        const hc_sr04_config_t *config)
{
    int rc;

    if (sensor == NULL)
    {
        return HC_SR04_INVALID_CONFIG;
    }

    if (!config_valid(config))
    {
        return HC_SR04_INVALID_CONFIG;
    }


    memset(
        sensor,
        0,
        sizeof(*sensor));


    sensor->cfg =
        *config;


    /*
     * Obtain QNX CPU cycle frequency.
     */
    sensor->cycles_per_sec =
        SYSPAGE_ENTRY(qtime)->cycles_per_sec;


    if (sensor->cycles_per_sec == 0)
    {
        return HC_SR04_INIT_ERROR;
    }


    /*
     * Enable I/O privilege.
     */
    rc =
        ThreadCtl(
            _NTO_TCTL_IO,
            0);

    if (rc == -1)
    {
        perror(
            "ThreadCtl(_NTO_TCTL_IO)");

        return HC_SR04_INIT_ERROR;
    }


    /*
     * Map BCM2711 GPIO controller.
     */
    sensor->gpio_base =
        mmap_device_memory(
            NULL,
            GPIO_MAP_SIZE,
            PROT_READ |
            PROT_WRITE |
            PROT_NOCACHE,
            0,
            GPIO_BASE_ADDRESS);


    if (sensor->gpio_base ==
        MAP_FAILED)
    {
        perror(
            "mmap_device_memory");

        sensor->gpio_base =
            NULL;

        return HC_SR04_INIT_ERROR;
    }


    /*
     * TRIG = OUTPUT
     */
    gpio_set_function(
        sensor,
        sensor->cfg.trig_gpio,
        GPIO_OUTPUT);


    /*
     * ECHO = INPUT
     */
    gpio_set_function(
        sensor,
        sensor->cfg.echo_gpio,
        GPIO_INPUT);


    /*
     * Disable pulls.
     */
    gpio_disable_pull(
        sensor,
        sensor->cfg.trig_gpio);

    gpio_disable_pull(
        sensor,
        sensor->cfg.echo_gpio);


    /*
     * Initial states.
     */
    gpio_low(
        sensor,
        sensor->cfg.trig_gpio);


    /*
     * Wait briefly for ECHO to settle.
     */
    delay_us(
        100,
        sensor->cycles_per_sec);


    /*
     * Clear any stuck ECHO state.
     */
    if (gpio_read(
            sensor,
            sensor->cfg.echo_gpio))
    {
        /*
         * We don't fail initialization immediately.
         *
         * The first measurement will perform recovery.
         */
    }


    /*
     * Initialize mutex.
     */
    rc =
        pthread_mutex_init(
            &sensor->mutex,
            NULL);

    if (rc != 0)
    {
        munmap_device_memory(
            (void *)sensor->gpio_base,
            GPIO_MAP_SIZE);

        sensor->gpio_base = NULL;

        return HC_SR04_INIT_ERROR;
    }


    sensor->initialized = true;


    return HC_SR04_OK;
}


/* ============================================================
 * DEINITIALIZATION
 * ============================================================ */

void hc_sr04_deinit(
        hc_sr04_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }


    if (!sensor->initialized)
    {
        return;
    }


    /*
     * Put TRIG LOW.
     */
    gpio_low(
        sensor,
        sensor->cfg.trig_gpio);


    sensor->initialized = false;


    pthread_mutex_destroy(
        &sensor->mutex);


    if (sensor->gpio_base != NULL)
    {
        munmap_device_memory(
            (void *)sensor->gpio_base,
            GPIO_MAP_SIZE);

        sensor->gpio_base = NULL;
    }
}


/* ============================================================
 * DISTANCE VALIDATION
 * ============================================================ */

bool hc_sr04_distance_valid(
        const hc_sr04_t *sensor,
        float distance_cm)
{
    if (sensor == NULL)
    {
        return false;
    }

    if (distance_cm <
        sensor->cfg.min_distance_cm)
    {
        return false;
    }

    if (distance_cm >
        sensor->cfg.max_distance_cm)
    {
        return false;
    }

    return true;
}


/* ============================================================
 * MAIN MEASUREMENT FUNCTION
 * ============================================================ */

hc_sr04_status_t hc_sr04_read(
        hc_sr04_t *sensor,
        float *distance_cm,
        uint64_t *echo_us)
{
    hc_sr04_status_t status;

    uint64_t rising_cycles;
    uint64_t falling_cycles;

    uint64_t pulse_us;

    float distance;


    /*
     * Validate arguments.
     */
    if (sensor == NULL ||
        distance_cm == NULL ||
        echo_us == NULL)
    {
        return HC_SR04_INVALID_CONFIG;
    }


    if (!sensor->initialized)
    {
        return HC_SR04_NOT_INITIALIZED;
    }


    /*
     * Make sure only one thread is measuring this sensor.
     */
    if (pthread_mutex_lock(
            &sensor->mutex) != 0)
    {
        return HC_SR04_SYSTEM_ERROR;
    }


    /*
     * Default outputs.
     *
     * IMPORTANT:
     *
     * Never return an old distance when a new measurement
     * fails.
     */
    *distance_cm = -1.0f;
    *echo_us = 0;


    /*
     * --------------------------------------------------------
     * STEP 1
     *
     * Make sure ECHO is LOW.
     * --------------------------------------------------------
     */

    status =
        wait_echo_low(sensor);

    if (status != HC_SR04_OK)
    {
        sensor->stats.total_measurements++;

        sensor->stats.echo_timeout_count++;

        pthread_mutex_unlock(
            &sensor->mutex);

        return status;
    }


    /*
     * --------------------------------------------------------
     * STEP 2
     *
     * Send trigger pulse.
     * --------------------------------------------------------
     */

    send_trigger(sensor);


    /*
     * --------------------------------------------------------
     * STEP 3
     *
     * Wait for rising ECHO edge.
     * --------------------------------------------------------
     */

    status =
        wait_for_echo_rising(
            sensor,
            &rising_cycles);


    if (status != HC_SR04_OK)
    {
        sensor->stats.total_measurements++;

        if (status ==
            HC_SR04_NO_ECHO)
        {
            sensor->stats.no_echo_count++;
        }
        else
        {
            sensor->stats.system_error_count++;
        }


        pthread_mutex_unlock(
            &sensor->mutex);

        return status;
    }


    /*
     * --------------------------------------------------------
     * STEP 4
     *
     * Wait for falling ECHO edge.
     * --------------------------------------------------------
     */

    status =
        wait_for_echo_falling(
            sensor,
            rising_cycles,
            &falling_cycles);


    if (status != HC_SR04_OK)
    {
        sensor->stats.total_measurements++;


        if (status ==
            HC_SR04_ECHO_TIMEOUT)
        {
            sensor->stats.echo_timeout_count++;
        }
        else if (status ==
                 HC_SR04_TOO_CLOSE)
        {
            sensor->stats.too_close_count++;
        }
        else if (status ==
                 HC_SR04_OUT_OF_RANGE)
        {
            sensor->stats.out_of_range_count++;
        }
        else
        {
            sensor->stats.system_error_count++;
        }


        pthread_mutex_unlock(
            &sensor->mutex);

        return status;
    }


    /*
     * --------------------------------------------------------
     * STEP 5
     *
     * Calculate exact pulse width.
     * --------------------------------------------------------
     */

    pulse_us =
        cycles_to_us(
            falling_cycles - rising_cycles,
            sensor->cycles_per_sec);


    *echo_us =
        pulse_us;


    /*
     * --------------------------------------------------------
     * STEP 6
     *
     * Calculate distance.
     *
     * distance =
     *       echo_time * speed_of_sound / 2
     * --------------------------------------------------------
     */

    distance =
        ((float)pulse_us *
         sensor->cfg.speed_of_sound_cm_per_us)
        / 2.0f;


    /*
     * --------------------------------------------------------
     * STEP 7
     *
     * Physical range validation.
     * --------------------------------------------------------
     */

    if (distance <
        sensor->cfg.min_distance_cm)
    {
        sensor->stats.total_measurements++;

        sensor->stats.too_close_count++;

        pthread_mutex_unlock(
            &sensor->mutex);

        return HC_SR04_TOO_CLOSE;
    }


    if (distance >
        sensor->cfg.max_distance_cm)
    {
        sensor->stats.total_measurements++;

        sensor->stats.out_of_range_count++;

        pthread_mutex_unlock(
            &sensor->mutex);

        return HC_SR04_OUT_OF_RANGE;
    }


    /*
     * Valid result.
     */
    *distance_cm =
        distance;


    stats_record_success(
        sensor,
        distance,
        pulse_us);


    pthread_mutex_unlock(
        &sensor->mutex);


    return HC_SR04_OK;
}


/* ============================================================
 * SIMPLE DISTANCE API
 * ============================================================ */

float hc_sr04_read_distance(
        hc_sr04_t *sensor)
{
    float distance;
    uint64_t echo_us;

    hc_sr04_status_t status;


    status =
        hc_sr04_read(
            sensor,
            &distance,
            &echo_us);


    if (status != HC_SR04_OK)
    {
        return -1.0f;
    }


    return distance;
}


/* ============================================================
 * WAIT BETWEEN MEASUREMENTS
 * ============================================================ */

void hc_sr04_wait_next_measurement(
        hc_sr04_t *sensor)
{
    struct timespec ts;

    uint64_t us;


    if (sensor == NULL)
    {
        return;
    }


    us =
        sensor->cfg.measurement_interval_us;


    ts.tv_sec =
        us / 1000000ULL;

    ts.tv_nsec =
        (long)((us % 1000000ULL) *
               1000ULL);


    nanosleep(
        &ts,
        NULL);
}


/* ============================================================
 * STATUS STRING
 * ============================================================ */

const char *hc_sr04_status_string(
        hc_sr04_status_t status)
{
    switch (status)
    {
        case HC_SR04_OK:
            return "OK";


        case HC_SR04_NO_ECHO:
            return "NO ECHO";


        case HC_SR04_ECHO_TIMEOUT:
            return "ECHO TIMEOUT";


        case HC_SR04_TOO_CLOSE:
            return "TOO CLOSE";


        case HC_SR04_OUT_OF_RANGE:
            return "OUT OF RANGE";


        case HC_SR04_INIT_ERROR:
            return "INIT ERROR";


        case HC_SR04_NOT_INITIALIZED:
            return "NOT INITIALIZED";


        case HC_SR04_INVALID_CONFIG:
            return "INVALID CONFIG";


        case HC_SR04_SYSTEM_ERROR:
            return "SYSTEM ERROR";


        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * GET STATISTICS
 * ============================================================ */

void hc_sr04_get_stats(
        hc_sr04_t *sensor,
        hc_sr04_stats_t *stats)
{
    if (sensor == NULL ||
        stats == NULL)
    {
        return;
    }


    if (sensor->initialized)
    {
        pthread_mutex_lock(
            &sensor->mutex);
    }


    *stats =
        sensor->stats;


    if (sensor->initialized)
    {
        pthread_mutex_unlock(
            &sensor->mutex);
    }
}


/* ============================================================
 * RESET STATISTICS
 * ============================================================ */

void hc_sr04_reset_stats(
        hc_sr04_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }


    if (sensor->initialized)
    {
        pthread_mutex_lock(
            &sensor->mutex);
    }


    memset(
        &sensor->stats,
        0,
        sizeof(sensor->stats));


    if (sensor->initialized)
    {
        pthread_mutex_unlock(
            &sensor->mutex);
    }
}


/* ============================================================
 * REAL-TIME THREAD
 * ============================================================ */

int hc_sr04_enable_realtime(void)
{
    struct sched_param param;

    int policy;

    int rc;


    /*
     * Get maximum FIFO priority.
     */
    param.sched_priority =
        sched_get_priority_max(
            SCHED_FIFO);


    if (param.sched_priority < 0)
    {
        return -1;
    }


    policy =
        SCHED_FIFO;


    rc =
        pthread_setschedparam(
            pthread_self(),
            policy,
            &param);


    if (rc != 0)
    {
        /*
         * This can fail if the application doesn't have
         * the required scheduling privilege.
         *
         * The driver still works without it.
         */
        return -1;
    }


    return 0;
}


/* ============================================================
 * SIMPLE API
 * ============================================================
 *
 * Internally wraps the full API (hc_sr04_init / hc_sr04_read /
 * hc_sr04_deinit) around a small fixed pool of hc_sr04_t
 * instances, so several sensors can each be driven through
 * their own handle while the caller only supplies GPIO pins
 * and calls read_distance(handle).
 */

typedef struct
{
    hc_sr04_t sensor;

    bool in_use;

    uint64_t last_read_cycles;

} hc_sr04_simple_slot_t;


static hc_sr04_simple_slot_t
    g_simple_slots[HC_SR04_SIMPLE_MAX_SENSORS];


int hc_sr04_begin(
        unsigned trig_gpio,
        unsigned echo_gpio)
{
    hc_sr04_config_t config;

    hc_sr04_status_t status;

    int handle;
    int i;


    /*
     * Find a free slot.
     */
    handle = -1;

    for (i = 0; i < HC_SR04_SIMPLE_MAX_SENSORS; i++)
    {
        if (!g_simple_slots[i].in_use)
        {
            handle = i;
            break;
        }
    }

    if (handle == -1)
    {
        /*
         * All slots in use.
         */
        return -1;
    }


    hc_sr04_config_default(
        &config);

    config.trig_gpio = trig_gpio;
    config.echo_gpio = echo_gpio;


    status =
        hc_sr04_init(
            &g_simple_slots[handle].sensor,
            &config);

    if (status != HC_SR04_OK)
    {
        return -1;
    }


    g_simple_slots[handle].in_use = true;

    g_simple_slots[handle].last_read_cycles = 0;


    return handle;
}


float read_distance(int handle)
{
    hc_sr04_simple_slot_t *slot;

    float distance;
    uint64_t echo_us;

    hc_sr04_status_t status;


    if (handle < 0 ||
        handle >= HC_SR04_SIMPLE_MAX_SENSORS)
    {
        return -1.0f;
    }


    slot = &g_simple_slots[handle];

    if (!slot->in_use)
    {
        return -1.0f;
    }


    /*
     * Enforce this sensor's configured measurement interval
     * automatically, so back-to-back calls never cause
     * ultrasonic cross-talk. Each handle tracks its own timing,
     * so reads on different sensors don't block each other.
     */
    if (slot->last_read_cycles != 0)
    {
        uint64_t elapsed_us =
            cycles_to_us(
                hc_cycles() - slot->last_read_cycles,
                slot->sensor.cycles_per_sec);

        if (elapsed_us <
            slot->sensor.cfg.measurement_interval_us)
        {
            struct timespec ts;

            uint64_t remaining_us =
                slot->sensor.cfg.measurement_interval_us -
                elapsed_us;

            ts.tv_sec =
                remaining_us / 1000000ULL;

            ts.tv_nsec =
                (long)((remaining_us % 1000000ULL) * 1000ULL);

            nanosleep(
                &ts,
                NULL);
        }
    }


    status =
        hc_sr04_read(
            &slot->sensor,
            &distance,
            &echo_us);

    slot->last_read_cycles =
        hc_cycles();


    if (status != HC_SR04_OK)
    {
        return -1.0f;
    }


    return distance;
}


void hc_sr04_end(int handle)
{
    if (handle < 0 ||
        handle >= HC_SR04_SIMPLE_MAX_SENSORS)
    {
        return;
    }


    if (g_simple_slots[handle].in_use)
    {
        hc_sr04_deinit(
            &g_simple_slots[handle].sensor);

        g_simple_slots[handle].in_use = false;
    }
}
