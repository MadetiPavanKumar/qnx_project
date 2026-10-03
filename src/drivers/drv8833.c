#include "drv8833.h"

#include <stddef.h>
#include <pthread.h>
#include <stdatomic.h>


/* ----------------------------------------------------------------------
   Internal state - same pattern as l298n.c: motor_config is written
   once under driver_lock during init and is read-only afterward,
   which is what lets drv8833_emergency_stop() read it lock-free. */

static drv8833_config_t motor_config;
static drv8833_status_t motor_status;

static atomic_bool driver_initialized = false;
static pthread_mutex_t driver_lock = PTHREAD_MUTEX_INITIALIZER;


/* ----------------------------------------------------------------------
   Validation
   ---------------------------------------------------------------------- */

static int validate_speed(unsigned int speed)
{
    if (speed > DRV8833_SPEED_MAX)
        return DRV8833_ERR_INVALID_ARG;

    return DRV8833_OK;
}

static int validate_config(const drv8833_config_t *config)
{
    int pins[5];
    int i, j;

    if (config == NULL)
        return DRV8833_ERR_INVALID_ARG;

    if (config->pwm_frequency == 0 ||
        config->pwm_frequency > DRV8833_MAX_PWM_FREQUENCY_HZ)
    {
        return DRV8833_ERR_INVALID_ARG;
    }

    pins[0] = config->en_pin;
    pins[1] = config->right_fwd_pin;
    pins[2] = config->right_rev_pin;
    pins[3] = config->left_fwd_pin;
    pins[4] = config->left_rev_pin;

    for (i = 0; i < 5; i++)
    {
        if (pins[i] < 0)
            return DRV8833_ERR_INVALID_ARG;
    }

    for (i = 0; i < 5; i++)
    {
        for (j = i + 1; j < 5; j++)
        {
            if (pins[i] == pins[j])
                return DRV8833_ERR_INVALID_ARG;
        }
    }

    return DRV8833_OK;
}


/* ----------------------------------------------------------------------
   Low-level hardware helpers - every pin write is attempted
   regardless of earlier failures, matching l298n.c's philosophy: a
   single failed write can never leave the driver silently half-set.
   ---------------------------------------------------------------------- */

static int set_right_dir(drv8833_dir_t dir)
{
    int rc1, rc2;

    switch (dir)
    {
        case DRV8833_DIR_FORWARD:
            rc1 = rpi_gpio_output(motor_config.right_fwd_pin, GPIO_HIGH);
            rc2 = rpi_gpio_output(motor_config.right_rev_pin, GPIO_LOW);
            break;

        case DRV8833_DIR_REVERSE:
            rc1 = rpi_gpio_output(motor_config.right_fwd_pin, GPIO_LOW);
            rc2 = rpi_gpio_output(motor_config.right_rev_pin, GPIO_HIGH);
            break;

        case DRV8833_DIR_STOP:
        default:
            rc1 = rpi_gpio_output(motor_config.right_fwd_pin, GPIO_LOW);
            rc2 = rpi_gpio_output(motor_config.right_rev_pin, GPIO_LOW);
            break;
    }

    return (rc1 == 0 && rc2 == 0) ? DRV8833_OK : DRV8833_ERR_HW;
}

/* Left motor's wiring is physically reversed relative to the right
   motor on this board - matches the tested reference code exactly. */
static int set_left_dir(drv8833_dir_t dir)
{
    int rc1, rc2;

    switch (dir)
    {
        case DRV8833_DIR_FORWARD:
            rc1 = rpi_gpio_output(motor_config.left_fwd_pin, GPIO_LOW);
            rc2 = rpi_gpio_output(motor_config.left_rev_pin, GPIO_HIGH);
            break;

        case DRV8833_DIR_REVERSE:
            rc1 = rpi_gpio_output(motor_config.left_fwd_pin, GPIO_HIGH);
            rc2 = rpi_gpio_output(motor_config.left_rev_pin, GPIO_LOW);
            break;

        case DRV8833_DIR_STOP:
        default:
            rc1 = rpi_gpio_output(motor_config.left_fwd_pin, GPIO_LOW);
            rc2 = rpi_gpio_output(motor_config.left_rev_pin, GPIO_LOW);
            break;
    }

    return (rc1 == 0 && rc2 == 0) ? DRV8833_OK : DRV8833_ERR_HW;
}

/* Force the hardware to a safe electrical state: PWM duty 0, all
   direction pins low. Reads motor_config WITHOUT the lock - safe
   because motor_config is immutable once driver_initialized is true
   (see header note on drv8833_emergency_stop()). */
static int force_stop_hardware(void)
{
    int rc = DRV8833_OK;

    if (rpi_gpio_set_pwm_duty_cycle(motor_config.en_pin, 0) != 0)
        rc = DRV8833_ERR_HW;

    if (rpi_gpio_output(motor_config.right_fwd_pin, GPIO_LOW) != 0)
        rc = DRV8833_ERR_HW;

    if (rpi_gpio_output(motor_config.right_rev_pin, GPIO_LOW) != 0)
        rc = DRV8833_ERR_HW;

    if (rpi_gpio_output(motor_config.left_fwd_pin, GPIO_LOW) != 0)
        rc = DRV8833_ERR_HW;

    if (rpi_gpio_output(motor_config.left_rev_pin, GPIO_LOW) != 0)
        rc = DRV8833_ERR_HW;

    return rc;
}


/* ----------------------------------------------------------------------
   Locked operations - caller already holds driver_lock.
   ---------------------------------------------------------------------- */

static int stop_locked(void)
{
    int rc = force_stop_hardware();

    motor_status.left_dir       = DRV8833_DIR_STOP;
    motor_status.right_dir      = DRV8833_DIR_STOP;
    motor_status.speed_percent  = 0;
    motor_status.enabled        = false;

    return rc;
}

static int apply_motion_locked(drv8833_dir_t left_dir, drv8833_dir_t right_dir,
                                unsigned int speed_percent)
{
    int rc = DRV8833_OK;

    if (set_left_dir(left_dir) != DRV8833_OK)
        rc = DRV8833_ERR_HW;

    if (set_right_dir(right_dir) != DRV8833_OK)
        rc = DRV8833_ERR_HW;

    /* Speed is shared across both channels - this is the hardware
       constraint documented in drv8833.h, not a bug. */
    if (rpi_gpio_set_pwm_duty_cycle(motor_config.en_pin, speed_percent) != 0)
        rc = DRV8833_ERR_HW;

    if (rc != DRV8833_OK)
    {
        /* Partial failure: don't report a commanded state that may
           not actually be true on the hardware - force a known-safe
           stop instead. */
        (void)stop_locked();
        return rc;
    }

    motor_status.left_dir      = left_dir;
    motor_status.right_dir     = right_dir;
    motor_status.speed_percent = speed_percent;
    motor_status.enabled       = (left_dir != DRV8833_DIR_STOP ||
                                   right_dir != DRV8833_DIR_STOP);

    return DRV8833_OK;
}


/* ----------------------------------------------------------------------
   Public API
   ---------------------------------------------------------------------- */

int drv8833_init(const drv8833_config_t *config)
{
    int rc = validate_config(config);
    if (rc != DRV8833_OK)
        return rc;

    if (atomic_load(&driver_initialized))
        return DRV8833_ERR_ALREADY_INIT;

    pthread_mutex_lock(&driver_lock);

    if (atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_ALREADY_INIT;
    }

    motor_config = *config;

    rc = rpi_gpio_setup(motor_config.en_pin, GPIO_OUT);
    if (rc == 0)
        rc = rpi_gpio_setup_pwm(motor_config.en_pin, motor_config.pwm_frequency,
                                 GPIO_PWM_MODE_MS);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_HW;
    }

    if (rpi_gpio_setup(motor_config.right_fwd_pin, GPIO_OUT) != 0 ||
        rpi_gpio_setup(motor_config.right_rev_pin, GPIO_OUT) != 0 ||
        rpi_gpio_setup(motor_config.left_fwd_pin,  GPIO_OUT) != 0 ||
        rpi_gpio_setup(motor_config.left_rev_pin,  GPIO_OUT) != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_HW;
    }

    /* All pins configured - mark initialized, then drive to a known-
       safe stopped state before returning control to the caller. */
    atomic_store(&driver_initialized, true);

    rc = stop_locked();

    pthread_mutex_unlock(&driver_lock);

    return rc;
}

int drv8833_forward(unsigned int speed_percent)
{
    int rc = validate_speed(speed_percent);
    if (rc != DRV8833_OK) return rc;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = apply_motion_locked(DRV8833_DIR_FORWARD, DRV8833_DIR_FORWARD, speed_percent);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_backward(unsigned int speed_percent)
{
    int rc = validate_speed(speed_percent);
    if (rc != DRV8833_OK) return rc;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = apply_motion_locked(DRV8833_DIR_REVERSE, DRV8833_DIR_REVERSE, speed_percent);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_left(unsigned int speed_percent)
{
    int rc = validate_speed(speed_percent);
    if (rc != DRV8833_OK) return rc;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = apply_motion_locked(DRV8833_DIR_REVERSE, DRV8833_DIR_FORWARD, speed_percent);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_right(unsigned int speed_percent)
{
    int rc = validate_speed(speed_percent);
    if (rc != DRV8833_OK) return rc;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = apply_motion_locked(DRV8833_DIR_FORWARD, DRV8833_DIR_REVERSE, speed_percent);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_set_motion(drv8833_dir_t left_dir, drv8833_dir_t right_dir,
                        unsigned int speed_percent)
{
    int rc = validate_speed(speed_percent);
    if (rc != DRV8833_OK) return rc;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = apply_motion_locked(left_dir, right_dir, speed_percent);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_stop(void)
{
    int rc;
    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = stop_locked();
    pthread_mutex_unlock(&driver_lock);
    return rc;
}

int drv8833_emergency_stop(void)
{
    int rc;

    /* Lock-free lifecycle check - safe because motor_config is
       immutable once driver_initialized is true (see header). */
    if (!atomic_load(&driver_initialized))
        return DRV8833_ERR_NOT_INITIALIZED;

    /* Force hardware safe WITHOUT waiting on driver_lock, so this
       can never be blocked by another thread mid-call. */
    rc = force_stop_hardware();

    /* Best-effort, non-blocking bookkeeping update. */
    if (pthread_mutex_trylock(&driver_lock) == 0)
    {
        motor_status.left_dir      = DRV8833_DIR_STOP;
        motor_status.right_dir     = DRV8833_DIR_STOP;
        motor_status.speed_percent = 0;
        motor_status.enabled       = false;

        pthread_mutex_unlock(&driver_lock);
    }

    return rc;
}

int drv8833_get_status(drv8833_status_t *status)
{
    if (status == NULL)
        return DRV8833_ERR_INVALID_ARG;

    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    *status = motor_status;
    pthread_mutex_unlock(&driver_lock);
    return DRV8833_OK;
}

int drv8833_deinit(void)
{
    int rc;
    pthread_mutex_lock(&driver_lock);
    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return DRV8833_ERR_NOT_INITIALIZED;
    }
    rc = stop_locked();
    atomic_store(&driver_initialized, false);
    pthread_mutex_unlock(&driver_lock);
    return rc;
}
