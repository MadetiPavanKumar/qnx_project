/*
 * sensor_monitor.c
 * ---------------------------------------------------------------
 * Reads the ultrasonic sensors every SENSOR_PERIOD_MS, cleans the raw
 * readings with one sensor_health_t per sensor (median filter + fault
 * logic, see sensor_health.c) and sends the CLEAN values plus a status
 * per sensor to the Safety Supervisor.
 *
 *   raw HC-SR04 reading --> sensor_health_update() --> filtered cm
 *                                                      + OK/FAULT status
 *                                                      --> MsgSend(Safety)
 *
 * Safety then only has to DECIDE; it never sees noisy readings.
 *
 * It is the LOWEST priority control task (10): the HC-SR04 driver
 * busy-waits for the echo, and we don't want that to starve
 * Navigation or Safety.
 */
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../sensors/sensor_monitor.h"
#include "../sensors/sensor_health.h"
#include "../safety/safety_supervisor.h"
#include "../system/watchdog.h"
#include "../system/ticker.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"
#include "../drivers/hc_sr04.h"

#define HANG_AFTER_MS     15000   /* when --test-hang freezes us       */
#define HANG_DURATION_S   3       /* ...for how long                   */

static int g_test_hang;
static int sensor_handle[NUM_SENSORS] = { -1, -1 };

/* log_event: print the log-worthy things sensor_health_update() reports. */
static void log_event(int sensor, sensor_event_t ev)
{
    switch (ev) {
    case SENSOR_EVENT_READY:     log_msg("[SENSOR] sensor %d ready", sensor); break;
    case SENSOR_EVENT_RECOVERED: log_msg("[SENSOR] sensor %d recovered", sensor); break;
    case SENSOR_EVENT_FAULT:
        log_msg("[SENSOR] sensor %d FAULT (%d bad readings in a row)", sensor, SENSOR_FAULT_AFTER);
        break;
    default: break;
    }
}

/* sensor_thread: periodic loop driven by a timer pulse.
 *   1. Wait for the tick pulse.
 *   2. For every sensor: read_distance() (blocking, a bit slow - that's
 *      why we are low priority; returns -1.0 on a failed measurement),
 *      feed the raw value to that sensor's sensor_health_t, copy the
 *      filtered distance + status into the message.
 *   3. MsgSend the message to Safety and wait for its ack.
 *   4. Send a heartbeat to the watchdog.                              */
static void *sensor_thread(void *arg)
{
    struct _pulse pulse;
    sensor_msg_t msg;
    ack_reply_t ack;
    sensor_health_t health[NUM_SENSORS];
    int tick_chid, safety_coid, wd, i;
    int hang_done = 0;
    uint64_t start = now_ms();
    (void)arg;

    for (i = 0; i < NUM_SENSORS; i++)
        sensor_health_init(&health[i]);

    /* A private channel just to receive our own timer pulses. */
    tick_chid = ChannelCreate(0);
    if (tick_chid == -1 || timer_pulse_start(tick_chid, PULSE_TICK, SENSOR_PERIOD_MS) != 0)
        return NULL;

    safety_coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_get_chid(), _NTO_SIDE_CHANNEL, 0);
    wd = watchdog_connect();
    if (safety_coid == -1) {
        perror("sensor: ConnectAttach(safety)");
        return NULL;
    }

    for (;;) {
        MsgReceive(tick_chid, &pulse, sizeof(pulse), NULL);   /* wait for tick */

        /* Fault-injection for the demo: freeze once, silently. */
        if (g_test_hang && !hang_done && (now_ms() - start) > HANG_AFTER_MS) {
            hang_done = 1;
            log_msg("[SENSOR] (test) simulating a hang for %d s", HANG_DURATION_S);
            sleep(HANG_DURATION_S);
        }

        msg.type = MSG_SENSOR_DATA;
        for (i = 0; i < NUM_SENSORS; i++) {
            float raw = read_distance(sensor_handle[i]);
            log_event(i, sensor_health_update(&health[i], raw));
            msg.distance_cm[i] = health[i].distance_cm;
            msg.status[i]      = health[i].status;
        }

        /* Timeout so a stuck Safety task can't freeze us too. */
        msg_send_timeout(safety_coid, &msg, sizeof(msg), &ack, sizeof(ack), TICK_MS * 2);

        watchdog_beat(wd, TASK_SENSOR);
    }
    return NULL;
}

/* sensor_monitor_start: open the two HC-SR04 sensors (hc_sr04_begin
 * returns a handle >= 0), then launch the thread. If a sensor cannot
 * be opened we refuse to start - driving blind is not an option. */
int sensor_monitor_start(int test_hang)
{
    g_test_hang = test_hang;

    sensor_handle[0] = hc_sr04_begin(PIN_US0_TRIG, PIN_US0_ECHO);
    sensor_handle[1] = hc_sr04_begin(PIN_US1_TRIG, PIN_US1_ECHO);
    if (sensor_handle[0] < 0 || sensor_handle[1] < 0) {
        fprintf(stderr, "sensor_monitor: hc_sr04_begin failed (h0=%d h1=%d)\n",
                sensor_handle[0], sensor_handle[1]);
        return -1;
    }
    return create_rt_thread(sensor_thread, NULL, PRIO_SENSOR, "sensor_mon");
}

/* sensor_monitor_close: give the driver slots back (shutdown only). */
void sensor_monitor_close(void)
{
    int i;
    for (i = 0; i < NUM_SENSORS; i++)
        if (sensor_handle[i] >= 0)
            hc_sr04_end(sensor_handle[i]);
}
