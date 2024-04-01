#include "CameraSocketListener.hpp"

#include "canmore/client_ids.h"

#include <iostream>
#include <net/if.h>
#include <stdexcept>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expected interface name argument" << std::endl;
        return 1;
    }

    int ifIdx = if_nametoindex(argv[1]);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    CameraSocketListener listener(3005, ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA);
    listener.run();
    return 0;
}
