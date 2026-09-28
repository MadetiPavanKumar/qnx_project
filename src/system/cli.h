#ifndef CLI_H
#define CLI_H

/* Returns non-zero while a manual emergency stop (from the CLI) is
   active. Safety Supervisor checks this every tick and, if set,
   forces EMERGENCY/0% regardless of what any sensor says - a manual
   kill switch sits above the automatic decision logic, same as a
   real e-stop button would. */
int cli_is_estop_active(void);

/* Blocking. Runs the interactive command loop on stdin. Call this
   LAST from main(), after every task has started - it replaces the
   old pause(). */
void cli_run(void);

#endif /* CLI_H */
