#include "tools/CanmoreImageTransmitter.hpp"

#include "canmore/client_ids.h"

#include <chrono>
#include <iostream>
#include <net/if.h>
#include <thread>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expected interface name argument" << std::endl;
        return 1;
    }

    int ifIdx = if_nametoindex(argv[1]);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    cv::VideoCapture cap(0);  // Declaring an object to capture stream of frames from default camera//
    if (!cap.isOpened()) {    // This section prompt an error message if no video stream is found//
        std::cout << "No video stream detected" << std::endl;
        return 1;
    }

    CanmoreImageTransmitter imageTx(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA);

    cv::Mat myImage;
    while (true) {
        uint8_t streamId;
        if (imageTx.transmitEnabled(streamId)) {
            cap >> myImage;
            cv::cvtColor(myImage, myImage, cv::COLOR_BGR2RGB);
            if (myImage.empty()) {
                std::cout << "Capture failed" << std::endl;
                break;
            }

            imageTx.transmitImage(myImage);
            uint8_t keypress = imageTx.getKeypress();
            if (keypress) {
                std::cout << "Keypress: '" << keypress << "'" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        else {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return 0;
}
