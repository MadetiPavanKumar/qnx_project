#ifndef DRV8833_H
#define DRV8833_H

#include <stdbool.h>
#include <rpi_gpio.h>

/*
 * ============================================================================
 * DRV8833 dual H-bridge motor driver (phase/enable wiring)
 * ============================================================================
 *
 * Scope: matches your tested drv8833-motor-driver.c wiring exactly -
 * ONE shared PWM enable/nSLEEP pin controls speed for BOTH motors,
 * while four separate GPIO pins (two per motor) set each motor's
 * direction independently.
 *
 * HARDWARE CONSTRAINT (not a software limitation - a wiring one):
 * Because speed is shared, the two wheels can be commanded to
 * different DIRECTIONS independently (pivot turns, stopping one
 * wheel while the other runs), but NOT different speeds at the same
 * time - there is only one duty cycle for the whole board. True
 * differential-speed steering (smooth arcs while both wheels move
 * forward at different speeds) is not possible with this wiring; it
 * would require PWM on each channel's own xIN pin instead of the
 * shared EN pin - a hardware change, not something this driver can
 * work around.
 *
 * Direction convention (matches your tested code exactly):
 *
 *      Right motor, right_fwd_pin/right_rev_pin:
 *          fwd=HIGH, rev=LOW   -> right motor forward
 *          fwd=LOW,  rev=HIGH  -> right motor reverse
 *
 *      Left motor, left_fwd_pin/left_rev_pin (wiring is REVERSED
 *      relative to the right motor on this board, same as your
 *      tested code):
 *          fwd=LOW,  rev=HIGH  -> left motor forward
 *          fwd=HIGH, rev=LOW   -> left motor reverse
 *
 * Thread safety: mirrors the l298n.c driver's contract. All public
 * functions are safe to call concurrently from multiple threads via
 * a single mutex, EXCEPT drv8833_emergency_stop(), documented below,
 * which deliberately does not wait on that mutex so a fault/watchdog
 * context can never be blocked by it.
 *
 * Instancing: single DRV8833 instance via internal static state, same
 * as l298n.c - matches a single-board-per-process QNX deployment.
 * ============================================================================
 */

typedef enum
{
    DRV8833_OK                  = 0,
    DRV8833_ERR_INVALID_ARG     = -1,
    DRV8833_ERR_NOT_INITIALIZED = -2,
    DRV8833_ERR_ALREADY_INIT    = -3,
    DRV8833_ERR_HW              = -4

} drv8833_err_t;

/* Per-wheel direction. Speed is shared across both wheels - see the
   hardware constraint note above. */
typedef enum
{
    DRV8833_DIR_STOP = 0,
    DRV8833_DIR_FORWARD,
    DRV8833_DIR_REVERSE

} drv8833_dir_t;

#define DRV8833_SPEED_MIN   0
#define DRV8833_SPEED_MAX   100

#define DRV8833_MAX_PWM_FREQUENCY_HZ   (1000000u)

typedef struct
{
    int en_pin;          /* shared PWM enable / nSLEEP pin */

    int right_fwd_pin;    /* right motor direction pin (AIN1) */
    int right_rev_pin;    /* right motor direction pin (AIN2) */

    int left_fwd_pin;     /* left motor direction pin (BIN1) */
    int left_rev_pin;     /* left motor direction pin (BIN2) */

    unsigned int pwm_frequency;    /* PWM frequency in Hz, must be > 0 */

} drv8833_config_t;

/*
 * Current commanded state. Reflects the last successfully commanded
 * state, not a measured one - no feedback path on this driver.
 */
typedef struct
{
    drv8833_dir_t left_dir;
    drv8833_dir_t right_dir;
    unsigned int  speed_percent;   /* shared across both wheels */
    bool          enabled;

} drv8833_status_t;


/*
 * Initialize the driver: validates config, configures all GPIO/PWM
 * pins, leaves the driver in a stopped, known-safe state.
 */
int drv8833_init(const drv8833_config_t *config);

/*
 * Stop the motors and deinitialize the driver.
 */
int drv8833_deinit(void);


/*
 * Basic movement helpers. speed_percent: DRV8833_SPEED_MIN..MAX.
 */
int drv8833_forward(unsigned int speed_percent);

int drv8833_backward(unsigned int speed_percent);

int drv8833_left(unsigned int speed_percent);   /* pivot: left reverse, right forward */

int drv8833_right(unsigned int speed_percent);  /* pivot: left forward, right reverse */


/*
 * General primitive: independent direction per wheel, one shared
 * speed - everything above is a thin convenience wrapper over this.
 * On a hardware failure partway through, fails safe (forces a
 * stopped state) rather than leaving pins in a partial/inconsistent
 * combination, and returns DRV8833_ERR_HW.
 */
int drv8833_set_motion(drv8833_dir_t left_dir, drv8833_dir_t right_dir,
                        unsigned int speed_percent);


/*
 * Normal stop: PWM duty 0, all direction pins driven low.
 */
int drv8833_stop(void);


/*
 * Emergency stop. Same electrical result as drv8833_stop(), but does
 * NOT wait on the driver mutex - callable from a fault/watchdog
 * context even while another thread is mid-way through a driver
 * call. Physical safety prioritized over bookkeeping consistency,
 * exactly matching l298n_emergency_stop()'s contract.
 */
int drv8833_emergency_stop(void);


/*
 * Get a consistent, mutex-protected snapshot of commanded state.
 */
int drv8833_get_status(drv8833_status_t *status);

#endif /* DRV8833_H */
