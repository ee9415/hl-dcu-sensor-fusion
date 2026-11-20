#include "pbv_dcu/socketcan_io.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

int pbv_socketcan_open(const char *if_name, bool non_blocking) {
    int fd = -1;
    int flags = 0;
    struct ifreq ifr;
    struct sockaddr_can addr;

    if (if_name == NULL) {
        return -1;
    }

    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1u);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (non_blocking) {
        flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }
    }

    return fd;
}

void pbv_socketcan_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

int pbv_socketcan_send_frame(int fd, const PbvCanFrame *frame) {
    struct can_frame raw;
    if (fd < 0 || frame == NULL || frame->dlc > 8u) {
        return -1;
    }

    memset(&raw, 0, sizeof(raw));
    raw.can_id = frame->id;
    raw.len = frame->dlc;
    memcpy(raw.data, frame->data, frame->dlc);
    if (write(fd, &raw, sizeof(raw)) != (ssize_t)sizeof(raw)) {
        return -1;
    }
    return 0;
}

bool pbv_socketcan_read_frame(int fd, PbvCanFrame *frame_out) {
    struct can_frame raw;
    ssize_t nread = -1;

    if (fd < 0 || frame_out == NULL) {
        return false;
    }

    nread = read(fd, &raw, sizeof(raw));
    if (nread < 0) {
        return false;
    }
    if (nread != (ssize_t)sizeof(raw)) {
        return false;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->id = raw.can_id;
    frame_out->dlc = raw.len;
    memcpy(frame_out->data, raw.data, raw.len);
    return true;
}
