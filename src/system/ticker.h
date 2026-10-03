/*
 * ticker.h - periodic timer that delivers pulses (see ticker.c)
 */
#ifndef TICKER_H
#define TICKER_H

/* Every period_ms, the kernel sends a pulse with the given code to the
 * given channel. Returns 0 on success, -1 on error. */
int timer_pulse_start(int chid, int pulse_code, int period_ms);

#endif
