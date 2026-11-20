#ifndef PBV_DCU_TIME_UTILS_H
#define PBV_DCU_TIME_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t pbv_now_monotonic_ms(void);
void pbv_sleep_ms(uint32_t sleep_ms);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_TIME_UTILS_H */
