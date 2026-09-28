#include <stddef.h>
#include "sensor_health.h"
#include "../common/config.h"

/* Valid physical range for HC-SR04. Anything outside this is either
   a timeout or an out-of-spec reading, never a real distance. */
#define SENSOR_MIN_VALID_CM   2.0f
#define SENSOR_MAX_VALID_CM   400.0f

void sensor_ctx_init(sensor_ctx_t *ctx)
{
    ctx->last_good_cm    = 0.0f;
    ctx->have_last_good  = 0;
    ctx->consecutive_bad = 0;
    ctx->health          = SENSOR_OK;
}

/* Simple insertion-sort median - fine for the small n (5) we use per tick. */
static float median_of(float *samples, int n)
{
    for (int i = 1; i < n; i++)
    {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key)
        {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }
    return samples[n / 2];
}

static void note_bad_sample(sensor_ctx_t *ctx)
{
    ctx->consecutive_bad++;

    if (ctx->consecutive_bad >= SENSOR_FAULT_AFTER)
        ctx->health = SENSOR_FAULT;
    else if (ctx->consecutive_bad >= SENSOR_DEGRADED_AFTER)
        ctx->health = SENSOR_DEGRADED;
}

static void note_good_sample(sensor_ctx_t *ctx, float distance_cm)
{
    ctx->last_good_cm   = distance_cm;
    ctx->have_last_good = 1;
    ctx->consecutive_bad = 0;
    ctx->health          = SENSOR_OK;
}

float sensor_ctx_process(sensor_ctx_t *ctx, const float *raw_samples, int n)
{
    float work[16]; /* n is always SAMPLES_PER_TICK; 16 is a safety margin */
    for (int i = 0; i < n; i++)
        work[i] = raw_samples[i];

    float median = median_of(work, n);
    int out_of_range = (median < SENSOR_MIN_VALID_CM || median > SENSOR_MAX_VALID_CM);

    if (out_of_range)
    {
        note_bad_sample(ctx);
        /* Nothing trustworthy to report - fail toward the last known
           good (close) reading rather than reporting "clear." If we
           have never had a good reading at all, report the minimum
           valid distance so the caller treats us as unsafe, not far. */
        return ctx->have_last_good ? ctx->last_good_cm : SENSOR_MIN_VALID_CM;
    }

    if (!ctx->have_last_good)
    {
        note_good_sample(ctx, median);
        return median;
    }

    float delta = median - ctx->last_good_cm;

    if (delta <= 0.0f)
    {
        /* Object got closer (or stayed the same) - trust this
           immediately. Quick to distrust "far", instant to trust
           "close" is the safe-by-default asymmetry we want. */
        note_good_sample(ctx, median);
        return median;
    }

    if (delta > MAX_PLAUSIBLE_DELTA_CM)
    {
        /* Suddenly much further away than physically plausible this
           tick - almost certainly a dropout, not a real change.
           Don't trust it yet; keep reporting the last good (closer,
           safer) distance and count this as a bad sample. */
        note_bad_sample(ctx);
        return ctx->last_good_cm;
    }

    /* A plausible move further away - accept it. */
    note_good_sample(ctx, median);
    return median;
}
