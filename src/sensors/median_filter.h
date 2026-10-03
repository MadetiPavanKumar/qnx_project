/*
 * median_filter.h - tiny median filter for noisy sensor readings
 * (see median_filter.c)
 */
#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "../common/config.h"

typedef struct {
    float values[MEDIAN_WINDOW];   /* circular buffer of recent readings */
    int   count;                   /* how many slots are filled (<=window) */
    int   next;                    /* slot the next reading will overwrite */
} median_filter_t;

/* Forget everything (empty window). */
void  mf_init(median_filter_t *mf);

/* Store one new GOOD reading (oldest one is overwritten when full). */
void  mf_add(median_filter_t *mf, float value);

/* 1 once at least MEDIAN_MIN_SAMPLES readings are stored, else 0. */
int   mf_ready(const median_filter_t *mf);

/* Median of the stored readings (0 if the window is empty). */
float mf_median(const median_filter_t *mf);

#endif
