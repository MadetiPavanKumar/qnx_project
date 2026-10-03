/*
 * safety_supervisor.h - the highest-priority task that can override
 * navigation (see safety_supervisor.c)
 */
#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

/* Creates the Safety channel and starts its thread. */
int safety_supervisor_start(void);

/* Channel id (Navigation and Sensor Monitor attach to it). */
int safety_get_chid(void);

#endif
