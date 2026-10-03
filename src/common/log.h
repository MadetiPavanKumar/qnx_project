/*
 * log.h - tiny thread-safe logger (see log.c)
 */
#ifndef LOG_H
#define LOG_H

/* Call once at start-up, before any other task logs. */
void log_init(void);

/* printf-style logging with a "[  12345 ms]" timestamp prefix. */
void log_msg(const char *fmt, ...);

#endif
