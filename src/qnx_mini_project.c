/*
 * main.c
 * ---------------------------------------------------------------
 * Start-up and shutdown of the whole system.
 *
 * Architecture (all tasks are threads of ONE QNX process, talking via
 * QNX message passing):
 *
 *   Sensor Monitor (prio 10) --MsgSend--> Safety Supervisor (prio 30)
 *      (readings are median-filtered inside the Sensor Monitor)
 *   Navigation     (prio 20) --MsgSend--> Safety Supervisor
 *   Safety Supervisor        --MsgSend--> Motor Controller (prio 25)
 *   Safety Supervisor        --pulse----> Display / OLED   (prio  5)
 *   every task               --pulse----> Watchdog         (prio 15)
 *
 * Usage:
 *   ./qnx_mini_project              normal run, Ctrl+C to stop
 *   ./qnx_mini_project --test-hang  freeze the Sensor Monitor once at 15 s
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "common/config.h"
#include "common/log.h"
#include "system/watchdog.h"
#include "display/display_task.h"
#include "motor/motor_controller.h"
#include "safety/safety_supervisor.h"
#include "sensors/sensor_monitor.h"
#include "navigation/navigation.h"

/* main:
 *   1. Parse the optional --test-hang flag and start the logger.
 *   2. Block SIGINT/SIGTERM in this thread BEFORE creating the others
 *      (threads inherit the signal mask) so only main handles Ctrl+C.
 *   3. Start the tasks. ORDER MATTERS: a task that is a "server" must
 *      create its channel before its clients try to attach:
 *          watchdog -> display -> motor -> safety -> sensor -> navigation
 *      motor_controller_start() and sensor_monitor_start() also open
 *      the real hardware, so a wiring/driver problem stops us here
 *      instead of the robot driving half-blind.
 *   4. Sleep in sigwait() until Ctrl+C, then stop the motors and exit. */
int main(int argc, char *argv[])
{
    int test_hang = (argc > 1 && strcmp(argv[1], "--test-hang") == 0);
    sigset_t set;
    int sig;

    log_init();
    log_msg("=== QNX Safety Supervisor (mini) starting ===");

    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    if (watchdog_start()          != 0 ||
        display_start()           != 0 ||
        motor_controller_start()  != 0 ||
        safety_supervisor_start() != 0 ||
        sensor_monitor_start(test_hang) != 0 ||
        navigation_start()        != 0) {
        fprintf(stderr, "failed to start a task / init hardware - exiting\n");
        motor_controller_close();
        sensor_monitor_close();
        return 1;
    }

    log_msg("all tasks running - press Ctrl+C to stop%s",
            test_hang ? " (fault test: sensor will hang at 15 s)" : "");

    sigwait(&set, &sig);            /* sleep until Ctrl+C / kill */

    log_msg("shutting down - stopping motors");
    motor_controller_close();
    sensor_monitor_close();
    return 0;
}
