#ifndef PBV_DCU_SOCKETCAN_IO_H
#define PBV_DCU_SOCKETCAN_IO_H

#include <stdbool.h>

#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int pbv_socketcan_open(const char *if_name, bool non_blocking);
void pbv_socketcan_close(int fd);
int pbv_socketcan_send_frame(int fd, const PbvCanFrame *frame);
bool pbv_socketcan_read_frame(int fd, PbvCanFrame *frame_out);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_SOCKETCAN_IO_H */
