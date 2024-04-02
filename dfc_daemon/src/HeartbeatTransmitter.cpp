#include "DFCDaemon.hpp"

#include "canmore/protocol.h"

#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/raw.h>
#include <string.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

HeartbeatTransmitter::HeartbeatTransmitter(int ifIndex, int clientId):
    ifIndex(ifIndex), clientId(clientId), socketFd(-1) {
    // Open socket
    // Using CAN broadcast manager so we don't need to spawn a whole thread for the heartbeat
    if ((socketFd = socket(PF_CAN, SOCK_DGRAM | SOCK_CLOEXEC, CAN_BCM)) < 0) {
        throw std::system_error(errno, std::generic_category(), "CAN BCM socket");
    }

    // Connect to the Broadcast Manager on the correct interface
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifIndex;

    if (connect(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(socketFd);
        socketFd = -1;
        throw std::system_error(errno, std::generic_category(), "BCM connect");
    }
}

void HeartbeatTransmitter::start() {
    // We need a frame for each count in the heartbeat
    const static size_t num_frames = 1 << CANMORE_HEARTBEAT_CNT_LENGTH;

    // Message Struct Foramt
    size_t heartbeat_msg_size = sizeof(bcm_msg_head) + num_frames * sizeof(can_frame);
    bcm_msg_head *heartbeat_msg = (bcm_msg_head *) malloc(heartbeat_msg_size);
    memset(heartbeat_msg, 0, heartbeat_msg_size);

    // Configure the repeating message
    // Will transmit every heartbeat interval, looping through all the frame indices
    heartbeat_msg->opcode = TX_SETUP;
    heartbeat_msg->flags = SETTIMER | STARTTIMER;
    heartbeat_msg->count = 0;
    heartbeat_msg->ival1 = { .tv_sec = 0, .tv_usec = 0 };
    heartbeat_msg->ival2 = { .tv_sec = CANMORE_HEARTBEAT_INTERVAL_MS / 1000,
                             .tv_usec = CANMORE_HEARTBEAT_INTERVAL_MS * 1000 };
    heartbeat_msg->can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, CANMORE_CHAN_HEARTBEAT);
    heartbeat_msg->nframes = num_frames;

    // Fill out heartbeat messages for each frame count
    for (size_t i = 0; i < num_frames; i++) {
        heartbeat_msg->frames[i].can_id = CANMORE_CALC_UTIL_ID_C2A(clientId, CANMORE_CHAN_HEARTBEAT);
        heartbeat_msg->frames[i].can_dlc = 1;
        heartbeat_msg->frames[i].data[0] =
            CANMORE_CALC_TITAN_HEARTBEAT_DATA(i, 0, CANMORE_CONTROL_INTERFACE_MODE_LINUX, 0, 0);
    }

    if (write(socketFd, heartbeat_msg, heartbeat_msg_size) < 0) {
        free(heartbeat_msg);
        throw std::system_error(errno, std::generic_category(), "BCM write");
    }
    free(heartbeat_msg);
}

HeartbeatTransmitter::~HeartbeatTransmitter() {
    if (socketFd >= 0)
        close(socketFd);
}
