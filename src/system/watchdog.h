/*
 * watchdog.h - heartbeat watchdog / recovery (see watchdog.c)
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "../common/messages.h"

/* Creates the watchdog channel and starts its thread. */
int watchdog_start(void);

/* Channel id (other tasks attach to it to send heartbeats). */
int watchdog_get_chid(void);

/* Connect to the watchdog; returns a coid to use with watchdog_beat(). */
int watchdog_connect(void);

/* Send a heartbeat pulse ("I'm alive") for this task. Never blocks. */
void watchdog_beat(int wd_coid, task_id_t who);

#endif
