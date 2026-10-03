/*
 * median_filter.c
 * ---------------------------------------------------------------
 * Why a MEDIAN and not an average?
 *   An average is dragged by a single spike: 100, 100, 400, 100, 100
 *   averages to 160 cm, which is wrong. The median of the same five
 *   numbers is 100 - the spike is simply thrown away. Ultrasonic
 *   sensors produce exactly this kind of occasional wild reading.
 *
 * Cost: the median reacts about half a window late to a REAL change
 * (2 readings = ~200 ms with a 100 ms sensor period). That delay is
 * small compared to the safety distances in config.h.
 */
#include "../sensors/median_filter.h"

/* mf_init: empty the window. Used at start-up and whenever a sensor
 * goes into FAULT, so old values can never leak into the recovery. */
void mf_init(median_filter_t *mf)
{
    mf->count = 0;
    mf->next  = 0;
}

/* mf_add: put the newest good reading into the circular buffer.
 * When the buffer is full the oldest reading is overwritten, so the
 * window always holds the most recent MEDIAN_WINDOW readings. */
void mf_add(median_filter_t *mf, float value)
{
    mf->values[mf->next] = value;
    mf->next = (mf->next + 1) % MEDIAN_WINDOW;
    if (mf->count < MEDIAN_WINDOW)
        mf->count++;
}

/* mf_ready: a median of one or two numbers is not trustworthy, so the
 * sensor only counts as "ready" after MEDIAN_MIN_SAMPLES good reads. */
int mf_ready(const median_filter_t *mf)
{
    return mf->count >= MEDIAN_MIN_SAMPLES;
}

/* mf_median: copy the stored values, sort the copy with a simple
 * insertion sort (at most 5 numbers, so it costs nothing) and return
 * the middle one. With an even count the upper-middle value is used. */
float mf_median(const median_filter_t *mf)
{
    float tmp[MEDIAN_WINDOW];
    int i, j;

    if (mf->count == 0)
        return 0.0f;

    for (i = 0; i < mf->count; i++)
        tmp[i] = mf->values[i];

    for (i = 1; i < mf->count; i++) {
        float key = tmp[i];
        j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[mf->count / 2];
}
