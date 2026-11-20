#include "pbv_dcu/time_utils.h"

#include <errno.h>
#include <time.h>

uint64_t pbv_now_monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0u;
    }
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

void pbv_sleep_ms(uint32_t sleep_ms) {
    struct timespec req;
    req.tv_sec = (time_t)(sleep_ms / 1000u);
    req.tv_nsec = (long)((sleep_ms % 1000u) * 1000000u);
    while (nanosleep(&req, &req) != 0 && errno == EINTR) {
    }
}
