/******************************************************************************
 *
 * Adapted from StackOverflow post:
 *   stackoverflow.com/questions/14676730/clock-gettime-on-raspberry-pi-with-c
 * Written by Yosh Kemu
 * Adapted by Byung-Cheol Cho
 *
 ******************************************************************************/

#include <sys/time.h>

typedef struct {
    struct timeval startTimeVal;
} timer_obj;

void timer_start(timer_obj* ctx) {
    gettimeofday(&ctx->startTimeVal, NULL);
}

double timer_elapsed(timer_obj* ctx) {
    // Get current time
    struct timeval nowTimeVal;
    gettimeofday(&nowTimeVal, NULL);

    // Compute difference
    return 1.0 * (double) (nowTimeVal.tv_sec - ctx->startTimeVal.tv_sec) +
        ((double) nowTimeVal.tv_usec -
         (double) ctx->startTimeVal.tv_usec) / 1E6;
}
