/*
 * config.h
 * ---------------------------------------------------------------
 * All the tunable numbers for the project live here so we never
 * have "magic numbers" scattered around the code. If something
 * behaves weirdly on the real robot, THIS is the first file to
 * look at.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---------- Timing ------------------------------------------------
 * TICK_MS is the heartbeat of the whole system. Navigation sends a
 * request every tick and Safety must answer inside the same tick
 * (that is our "deadline"). */
#define TICK_MS                 75

/* The HC-SR04 driver already waits ~65 ms between two reads of the
 * same sensor, so reading both sensors takes > 100 ms. We give the
 * Sensor Monitor its own (slower) period instead of the tick. */
#define SENSOR_PERIOD_MS        100

/* ---------- Distance thresholds (cm) ------------------------------
 *  distance >  WARNING_CM         -> full speed
 *  EMERGENCY_CM < d <= WARNING_CM -> speed scaled down linearly
 *  distance <= EMERGENCY_CM       -> hard stop                     */
#define WARNING_CM              25.0f
#define EMERGENCY_CM            15.0f

/* Readings outside this window are treated as garbage (no echo,
 * timeout, out of range...). The driver returns -1 on failure.   */
#define MIN_VALID_CM            2.0f
#define MAX_VALID_CM            400.0f

/* Never command a speed below this (but above 0) - the motors just
 * hum and stall at very low PWM duty.                             */
#define MIN_MOVE_SPEED          60

/* ---------- Ultrasonic filtering + fault handling -------------------
 * Cheap HC-SR04 modules are noisy: now and then one reading is a wild
 * spike or a dropout (-1). We do NOT want one bad reading to decide
 * anything, so each sensor keeps its last few GOOD readings and the
 * value used by Safety is their MEDIAN (see sensors/median_filter.c).
 *
 *  MEDIAN_WINDOW        how many good readings are remembered
 *  MEDIAN_MIN_SAMPLES   a sensor is "ready" only once it has this many
 *                       good readings (also = good readings in a row
 *                       needed to recover after a fault)
 *  SENSOR_FAULT_AFTER   bad readings IN A ROW before a sensor is FAULT.
 *                       A single dropout is simply ignored (the median
 *                       of the older good readings is kept). With a
 *                       100 ms sensor period, 10 = one full second of
 *                       nothing usable.                                */
#define MEDIAN_WINDOW           15
#define MEDIAN_MIN_SAMPLES      10
#define SENSOR_FAULT_AFTER      15

/* If the newest sensor data is older than this, Safety stops trusting
 * it (the Sensor Monitor may be hung).                              */
#define SENSOR_STALE_MS         400

/* Safety must finish "decide + tell motors" within this many ms.    */
#define SAFETY_DEADLINE_MS      TICK_MS

/* Motor Controller dead-man timer: if NOTHING arrives for this long
 * it stops the motors by itself.                                    */
#define MOTOR_TIMEOUT_MS        (TICK_MS * 3)

/* ---------- Watchdog ---------------------------------------------- */
#define WATCHDOG_CHECK_MS           100
#define HEARTBEAT_TIMEOUT_MS        400   /* normal tasks              */
#define SENSOR_HEARTBEAT_TIMEOUT_MS 500   /* sensor reads are slower   */

/* ---------- Navigation behaviour ---------------------------------- */
#define NAV_CRUISE_SPEED        60      /* % PWM when driving forward  */
#define NAV_TURN_SPEED          50      /* % PWM when pivoting         */
#define NAV_BLOCKED_TICKS       5       /* ticks of EMERGENCY before we
                                           decide to turn away         */
#define NAV_TURN_TICKS          5       /* how long to pivot           */

/* ---------- Thread priorities (SCHED_FIFO) -------------------------
 * Higher number = higher priority on QNX. Safety is highest so it
 * can always pre-empt everything else - that is the "priority
 * override" requirement. Sensor Monitor is lowest of the control
 * tasks because its driver busy-waits on the ultrasonic echo.      */
#define PRIO_SAFETY             30
#define PRIO_MOTOR              25
#define PRIO_NAVIGATION         20
#define PRIO_WATCHDOG           15
#define PRIO_SENSOR             10
#define PRIO_DISPLAY            5       /* OLED = least important task */

/* ---------- OLED ---------------------------------------------------
 * The display runs in its own low-priority thread. Safety only drops a
 * tiny non-blocking PULSE for it (at most every DISPLAY_PERIOD_MS, or
 * immediately when the state changes), so a slow I2C bus can never
 * delay a safety decision.                                           */
#define OLED_I2C_DEVICE         "/dev/i2c1"
#define DISPLAY_PERIOD_MS       200

/* ---------- GPIO pins (BCM numbering, Raspberry Pi 4) -------------- */
#define NUM_SENSORS             2

#define PIN_US0_TRIG            5
#define PIN_US0_ECHO            6
#define PIN_US1_TRIG            16
#define PIN_US1_ECHO            19

#define PIN_MOTOR_EN            18      /* shared PWM enable (speed)   */
#define PIN_MOTOR_RIGHT_FWD     10
#define PIN_MOTOR_RIGHT_REV     9
#define PIN_MOTOR_LEFT_FWD      8
#define PIN_MOTOR_LEFT_REV      11
#define MOTOR_PWM_HZ            1000

#endif /* CONFIG_H */
