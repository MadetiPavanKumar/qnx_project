#include "l298n.h"

#include <stddef.h>
#include <pthread.h>
#include <stdatomic.h>


/*
 * ----------------------------------------------------------------------
 * Internal driver state
 * ----------------------------------------------------------------------
 *
 * motor_config is written once, under driver_lock, during l298n_init(),
 * and is not modified again until l298n_deinit() clears
 * driver_initialized. While driver_initialized is true it is
 * effectively read-only, which is what allows l298n_emergency_stop()
 * to read it without taking driver_lock (see header for the full
 * rationale).
 */

static l298n_config_t motor_config;

static l298n_status_t motor_status;

/*
 * atomic_bool so l298n_emergency_stop() can check driver lifecycle
 * state without taking driver_lock (and therefore without risking
 * being blocked by another in-progress call).
 */
static atomic_bool driver_initialized = false;

/*
 * Protects motor_config (during init/deinit) and motor_status against
 * concurrent access from multiple threads -- e.g. a periodic motor
 * control task and a fault/watchdog task. Not held by
 * l298n_emergency_stop()'s hardware-safing path; see header.
 */
static pthread_mutex_t driver_lock = PTHREAD_MUTEX_INITIALIZER;


/*
 * ----------------------------------------------------------------------
 * Validation helpers
 * ----------------------------------------------------------------------
 */

static int validate_speed_unsigned(int speed)
{
    if (speed < 0 || speed > L298N_SPEED_MAX)
        return L298N_ERR_INVALID_ARG;

    return L298N_OK;
}


static int validate_speed_signed(int speed)
{
    if (speed < L298N_SPEED_MIN || speed > L298N_SPEED_MAX)
        return L298N_ERR_INVALID_ARG;

    return L298N_OK;
}


/*
 * All configured pins must be non-negative and distinct from one
 * another. Catches copy-paste configuration errors (e.g. in1 == in3)
 * at init time instead of letting them silently misbehave. Deliberately
 * platform-agnostic: it does not assume a specific SoC's valid GPIO
 * numbering range.
 */

static int validate_config(const l298n_config_t *config)
{
    int pins[6];
    int i;
    int j;

    if (config == NULL)
        return L298N_ERR_INVALID_ARG;

    if (config->pwm_frequency == 0 ||
        config->pwm_frequency > L298N_MAX_PWM_FREQUENCY_HZ)
    {
        return L298N_ERR_INVALID_ARG;
    }

    pins[0] = config->in1;
    pins[1] = config->in2;
    pins[2] = config->in3;
    pins[3] = config->in4;
    pins[4] = config->ena;
    pins[5] = config->enb;

    for (i = 0; i < 6; i++)
    {
        if (pins[i] < 0)
            return L298N_ERR_INVALID_ARG;
    }

    for (i = 0; i < 6; i++)
    {
        for (j = i + 1; j < 6; j++)
        {
            if (pins[i] == pins[j])
                return L298N_ERR_INVALID_ARG;
        }
    }

    return L298N_OK;
}


/*
 * ----------------------------------------------------------------------
 * Low-level hardware helpers
 * ----------------------------------------------------------------------
 *
 * Every GPIO/PWM call's return value is checked. These helpers always
 * attempt every pin write they own (they do not early-return on the
 * first failure) so a single failed pin can never leave the H-bridge
 * in an undefined half-written state -- the caller gets L298N_ERR_HW
 * if anything failed, but every write was still attempted.
 */

static int set_left_direction(bool forward)
{
    int rc1;
    int rc2;

    if (forward)
    {
        rc1 = rpi_gpio_output(motor_config.in1, GPIO_HIGH);
        rc2 = rpi_gpio_output(motor_config.in2, GPIO_LOW);
    }
    else
    {
        rc1 = rpi_gpio_output(motor_config.in1, GPIO_LOW);
        rc2 = rpi_gpio_output(motor_config.in2, GPIO_HIGH);
    }

    return (rc1 == 0 && rc2 == 0) ? L298N_OK : L298N_ERR_HW;
}


static int set_right_direction(bool forward)
{
    int rc1;
    int rc2;

    if (forward)
    {
        rc1 = rpi_gpio_output(motor_config.in3, GPIO_HIGH);
        rc2 = rpi_gpio_output(motor_config.in4, GPIO_LOW);
    }
    else
    {
        rc1 = rpi_gpio_output(motor_config.in3, GPIO_LOW);
        rc2 = rpi_gpio_output(motor_config.in4, GPIO_HIGH);
    }

    return (rc1 == 0 && rc2 == 0) ? L298N_OK : L298N_ERR_HW;
}


/*
 * Force both motors to a safe electrical state: PWM duty 0, all
 * direction pins low. Reads motor_config WITHOUT taking driver_lock
 * (see header comment on l298n_emergency_stop() for why); this is
 * safe because motor_config is immutable for as long as
 * driver_initialized is true, which the caller has already checked.
 *
 * Every pin is attempted regardless of earlier failures, so a single
 * failed write can never mask the attempt to safe the other pins.
 */

static int force_stop_hardware(void)
{
    int rc = L298N_OK;

    if (rpi_gpio_set_pwm_duty_cycle(motor_config.ena, 0) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_set_pwm_duty_cycle(motor_config.enb, 0) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_output(motor_config.in1, GPIO_LOW) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_output(motor_config.in2, GPIO_LOW) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_output(motor_config.in3, GPIO_LOW) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_output(motor_config.in4, GPIO_LOW) != 0)
        rc = L298N_ERR_HW;

    return rc;
}


/*
 * ----------------------------------------------------------------------
 * Locked driver operations
 * ----------------------------------------------------------------------
 * Everything below this point assumes driver_lock is already held by
 * the caller.
 */

/*
 * Stop the motors and update bookkeeping. Used by l298n_stop(),
 * l298n_init()'s safe-start, and as the fail-safe path when
 * apply_motor_speed_locked() hits a hardware error partway through.
 */

static int stop_locked(void)
{
    int rc = force_stop_hardware();

    motor_status.left_speed = 0;
    motor_status.right_speed = 0;
    motor_status.enabled = false;

    return rc;
}


/*
 * Core differential-drive implementation. left_speed / right_speed
 * are signed (L298N_SPEED_MIN..L298N_SPEED_MAX): sign selects
 * direction per wheel, magnitude selects PWM duty cycle. Caller has
 * already validated the range.
 *
 * On any hardware failure, fails safe by forcing a stopped state
 * rather than leaving a partially-applied direction/duty combination,
 * and returns L298N_ERR_HW.
 */

static int apply_motor_speed_locked(int left_speed, int right_speed)
{
    int rc = L298N_OK;
    int left_duty = (left_speed >= 0) ? left_speed : -left_speed;
    int right_duty = (right_speed >= 0) ? right_speed : -right_speed;

    if (set_left_direction(left_speed >= 0) != L298N_OK)
        rc = L298N_ERR_HW;

    if (set_right_direction(right_speed >= 0) != L298N_OK)
        rc = L298N_ERR_HW;

    if (rpi_gpio_set_pwm_duty_cycle(motor_config.ena, left_duty) != 0)
        rc = L298N_ERR_HW;

    if (rpi_gpio_set_pwm_duty_cycle(motor_config.enb, right_duty) != 0)
        rc = L298N_ERR_HW;

    if (rc != L298N_OK)
    {
        /*
         * Partial failure: direction and/or duty cycle may be
         * inconsistent across the two H-bridges. Do not report
         * the requested speed as commanded -- force a known-safe
         * stop instead and let the caller decide how to recover.
         */
        (void)stop_locked();
        return rc;
    }

    motor_status.left_speed = left_speed;
    motor_status.right_speed = right_speed;
    motor_status.enabled = (left_speed != 0 || right_speed != 0);

    return L298N_OK;
}


/*
 * ----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------
 */

int l298n_init(const l298n_config_t *config)
{
    int rc;

    rc = validate_config(config);
    if (rc != L298N_OK)
        return rc;

    if (atomic_load(&driver_initialized))
        return L298N_ERR_ALREADY_INIT;

    pthread_mutex_lock(&driver_lock);

    /* Re-check under the lock: another thread may have raced us. */
    if (atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_ALREADY_INIT;
    }

    motor_config = *config;

    rc = rpi_gpio_setup_pwm(motor_config.ena, motor_config.pwm_frequency,
                             GPIO_PWM_MODE_MS);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    rc = rpi_gpio_setup_pwm(motor_config.enb, motor_config.pwm_frequency,
                             GPIO_PWM_MODE_MS);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    /*
     * Configure direction pins. driver_initialized is never set on a
     * failure path below, so even though pins configured so far are
     * left as GPIO outputs, no caller can command motion through
     * this driver instance until a full, successful l298n_init().
     */

    rc = rpi_gpio_setup(motor_config.in1, GPIO_OUT);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    rc = rpi_gpio_setup(motor_config.in2, GPIO_OUT);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    rc = rpi_gpio_setup(motor_config.in3, GPIO_OUT);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    rc = rpi_gpio_setup(motor_config.in4, GPIO_OUT);
    if (rc != 0)
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_HW;
    }

    /*
     * All pins configured. Mark the driver initialized and drive it
     * to a known-safe stopped state before returning control to the
     * caller -- motors are guaranteed stopped at this point.
     */

    atomic_store(&driver_initialized, true);

    rc = stop_locked();

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_forward(int speed)
{
    int rc = validate_speed_unsigned(speed);
    if (rc != L298N_OK)
        return rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = apply_motor_speed_locked(speed, speed);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_backward(int speed)
{
    int rc = validate_speed_unsigned(speed);
    if (rc != L298N_OK)
        return rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = apply_motor_speed_locked(-speed, -speed);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_left(int speed)
{
    int rc = validate_speed_unsigned(speed);
    if (rc != L298N_OK)
        return rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = apply_motor_speed_locked(-speed, speed);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_right(int speed)
{
    int rc = validate_speed_unsigned(speed);
    if (rc != L298N_OK)
        return rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = apply_motor_speed_locked(speed, -speed);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_set_motor_speed(int left_speed, int right_speed)
{
    int rc = validate_speed_signed(left_speed);
    if (rc != L298N_OK)
        return rc;

    rc = validate_speed_signed(right_speed);
    if (rc != L298N_OK)
        return rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = apply_motor_speed_locked(left_speed, right_speed);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_stop(void)
{
    int rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    rc = stop_locked();

    pthread_mutex_unlock(&driver_lock);

    return rc;
}


int l298n_emergency_stop(void)
{
    int rc;

    /*
     * Lock-free lifecycle check: driver_initialized only ever
     * transitions under driver_lock, and once true, motor_config is
     * immutable until l298n_deinit() runs -- so it is safe to read
     * motor_config below without taking the lock.
     */
    if (!atomic_load(&driver_initialized))
        return L298N_ERR_NOT_INITIALIZED;

    /*
     * Force the hardware safe immediately. This does NOT wait on
     * driver_lock, so it cannot be blocked by another thread that is
     * mid-way through a driver call (e.g. stuck in a slow GPIO/PWM
     * operation). Physical safety takes priority over bookkeeping.
     */
    rc = force_stop_hardware();

    /*
     * Best-effort, non-blocking attempt to keep l298n_get_status()
     * consistent with reality. If the lock is currently held
     * elsewhere, we do not wait for it -- the hardware is already
     * safe regardless of whether this succeeds.
     */
    if (pthread_mutex_trylock(&driver_lock) == 0)
    {
        motor_status.left_speed = 0;
        motor_status.right_speed = 0;
        motor_status.enabled = false;

        pthread_mutex_unlock(&driver_lock);
    }

    return rc;
}


int l298n_get_status(l298n_status_t *status)
{
    if (status == NULL)
        return L298N_ERR_INVALID_ARG;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    *status = motor_status;

    pthread_mutex_unlock(&driver_lock);

    return L298N_OK;
}


int l298n_deinit(void)
{
    int rc;

    pthread_mutex_lock(&driver_lock);

    if (!atomic_load(&driver_initialized))
    {
        pthread_mutex_unlock(&driver_lock);
        return L298N_ERR_NOT_INITIALIZED;
    }

    /* Always stop before releasing the driver. */
    rc = stop_locked();

    atomic_store(&driver_initialized, false);

    pthread_mutex_unlock(&driver_lock);

    return rc;
}
