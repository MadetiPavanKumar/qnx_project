#include <stdio.h>

#include "common/logger.h"
#include "system/recovery_manager.h"
#include "system/telemetry_service.h"
#include "system/cli.h"
#include "motor/motor_controller.h"
#include "safety/safety_supervisor.h"
#include "sensors/sensor_monitor.h"
#include "navigation/navigation.h"
#include "display/display_task.h"

int main(void)
{
    printf("=== Autonomous Vehicle Safety Supervisor ===\n");

    /* Logger first, before anything else that might log - recovery_
       manager_start() included, since it logs during its own
       startup. */
    logger_start();

    recovery_manager_start();
    motor_controller_start();
    safety_supervisor_start();
    /* Display owns all OLED/I2C access on its own low-priority
       thread now - it only reads telemetry_get(), so it has no
       ordering dependency on the other tasks. Safety Supervisor no
       longer touches the OLED directly (see safety_supervisor.c). */
    display_task_start();
    sensor_monitor_start();
    navigation_start();

    /* Telemetry Service is the cloud-facing observer - see
       telemetry_service.h. It has no ordering dependency on any
       other task either (it only ever reads their public getters),
       so where it starts relative to them doesn't matter; started
       last here simply because it's the least critical. */
    telemetry_service_start();

    /* Blocking - runs the interactive CLI on this thread for the
       rest of the process's life. */
    cli_run();

    return 0;
}
