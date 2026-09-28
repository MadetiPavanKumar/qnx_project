#ifndef L298N_H
#define L298N_H

#include <stdbool.h>
#include <rpi_gpio.h>


/*
 * ============================================================================
 * L298N dual H-bridge motor driver
 * ============================================================================
 *
 * Scope: this module drives ONE L298N board controlling a two-motor
 * differential (skid-steer) rover. It owns GPIO/PWM I/O only.
 *
 * It intentionally contains NO navigation, obstacle avoidance, PID/
 * closed-loop control, route planning, or sensor processing. Those
 * belong in a separate motor-control / navigation task that calls
 * into this driver.
 *
 * Direction convention (must match how the motors are physically
 * wired to the L298N outputs -- verify on your hardware):
 *
 *      Left motor,  IN1/IN2:
 *          IN1 = HIGH, IN2 = LOW   -> left motor spins forward
 *          IN1 = LOW,  IN2 = HIGH  -> left motor spins backward
 *
 *      Right motor, IN3/IN4:
 *          IN3 = HIGH, IN4 = LOW   -> right motor spins forward
 *          IN3 = LOW,  IN4 = HIGH  -> right motor spins backward
 *
 *      "Forward" for the rover means both motors commanded forward.
 *      A pivot turn means the two motors are commanded with opposite
 *      sign (e.g. left = -50, right = +50 spins in place).
 *
 * Thread safety:
 *      All public functions are safe to call concurrently from
 *      multiple threads (e.g. a periodic motor-control task and an
 *      asynchronous fault/watchdog task). Internally this is done
 *      with a single mutex guarding driver state, EXCEPT
 *      l298n_emergency_stop(), which is documented separately below
 *      because it deliberately does not wait on that mutex.
 *
 *      l298n_init() and l298n_deinit() must not be called
 *      concurrently with each other or while any other l298n_*
 *      call for the same driver instance is in flight -- they
 *      change driver lifecycle state, not just motor state.
 *
 * Instancing:
 *      This driver controls a single L298N instance via internal
 *      static state (matching typical QNX single-board-per-process
 *      robot deployments). All hardware pins are supplied through
 *      l298n_config_t at init time -- none are hard-coded.
 * ============================================================================
 */


/*
 * Return / error codes used by every function in this module.
 */
typedef enum
{
    L298N_OK                  = 0,   /* success                              */
    L298N_ERR_INVALID_ARG     = -1,  /* NULL pointer or out-of-range value   */
    L298N_ERR_NOT_INITIALIZED = -2,  /* driver not initialized               */
    L298N_ERR_ALREADY_INIT    = -3,  /* l298n_init() called while running    */
    L298N_ERR_HW              = -4   /* underlying GPIO/PWM call failed      */

} l298n_err_t;


/*
 * Commanded motor speed range (percent duty cycle, signed).
 *
 *      positive -> forward
 *      negative -> backward
 *      zero     -> stopped
 */
#define L298N_SPEED_MIN   (-100)
#define L298N_SPEED_MAX   (100)

/*
 * Sanity bound on configured PWM frequency. This is not a hardware
 * limit -- it exists purely to catch obviously-wrong configuration
 * (e.g. a units mistake passing Hz where kHz was meant) at init time
 * rather than silently passing garbage to the HAL.
 */
#define L298N_MAX_PWM_FREQUENCY_HZ   (1000000u)


/*
 * L298N hardware configuration.
 *
 * All GPIO/PWM pin assignments are caller-supplied; the driver never
 * hard-codes a pin number.
 */
typedef struct
{
    int in1;    /* left motor direction pin  */
    int in2;    /* left motor direction pin  */

    int in3;    /* right motor direction pin */
    int in4;    /* right motor direction pin */

    int ena;    /* left motor PWM enable pin  */
    int enb;    /* right motor PWM enable pin */

    unsigned int pwm_frequency;    /* PWM frequency in Hz, must be > 0 */

} l298n_config_t;


/*
 * Current commanded motor state.
 *
 * This reflects the last successfully commanded state, not a
 * measured/sensed one -- the driver has no feedback path (no
 * encoders, no current sensing). left_speed / right_speed are
 * signed, using the same convention as l298n_set_motor_speed().
 */
typedef struct
{
    int left_speed;
    int right_speed;

    bool enabled;

} l298n_status_t;


/*
 * Initialize the L298N driver: validates config, configures all
 * GPIO/PWM pins, and leaves the driver in a stopped, known-safe
 * state. Motors are guaranteed stopped both while this is in
 * progress and if it fails partway through.
 *
 * Must not be called while the driver is already initialized --
 * call l298n_deinit() first to reconfigure.
 *
 * Returns L298N_OK on success, or a negative l298n_err_t value.
 */
int l298n_init(const l298n_config_t *config);


/*
 * Stop the motors and deinitialize the driver.
 */
int l298n_deinit(void);


/*
 * Basic movement helpers. speed magnitude range: 0 to L298N_SPEED_MAX.
 */
int l298n_forward(int speed);

int l298n_backward(int speed);

int l298n_left(int speed);   /* pivot: left motor backward, right forward */

int l298n_right(int speed);  /* pivot: left motor forward, right backward */


/*
 * Differential drive -- the primary motor-control entry point.
 *
 * left_speed / right_speed: L298N_SPEED_MIN..L298N_SPEED_MAX, signed.
 * Sign selects direction per wheel independently, so arbitrary
 * steering and pivot turns (opposite signs) are supported.
 *
 * On a hardware (GPIO/PWM) failure partway through applying the
 * command, the driver fails safe: it forces both motors to a
 * stopped state rather than leaving direction and duty cycle
 * partially applied, and returns L298N_ERR_HW.
 */
int l298n_set_motor_speed(int left_speed, int right_speed);


/*
 * Normal stop: PWM duty set to 0 and all direction pins driven low.
 * Requires the driver to be initialized.
 */
int l298n_stop(void);


/*
 * Emergency stop.
 *
 * Electrically this reaches the same safe state as l298n_stop():
 * PWM duty 0, all direction pins low. It is a SEPARATE code path
 * with a different concurrency contract, intended to be callable
 * from a fault/watchdog context even while another thread may be
 * holding the driver lock (e.g. blocked inside a slow GPIO call):
 *
 *   - It writes the hardware to a safe state WITHOUT waiting on the
 *     driver mutex, so it cannot be blocked by another in-progress
 *     driver call.
 *   - It then makes a single best-effort, non-blocking attempt to
 *     also update the bookkeeping returned by l298n_get_status().
 *     If that attempt cannot acquire the lock immediately, the
 *     hardware is still guaranteed to be safe, but the status
 *     snapshot may briefly continue to show the pre-stop command
 *     until the next successful stop/command call.
 *
 * Physical safety of the motors is prioritized over bookkeeping
 * consistency. Returns L298N_ERR_NOT_INITIALIZED if the driver was
 * never initialized, L298N_ERR_HW if any underlying GPIO/PWM call
 * failed (the function still attempts every pin regardless), and
 * L298N_OK otherwise.
 */
int l298n_emergency_stop(void);


/*
 * Get a consistent, atomic snapshot of the current commanded motor
 * state (mutex-protected: never returns a torn read).
 */
int l298n_get_status(l298n_status_t *status);


#endif /* L298N_H */
