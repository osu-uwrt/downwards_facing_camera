#include "DFCMicroROSClient.hpp"

#include "canmore/client_ids.h"

#include <iostream>
#include <net/if.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

void testThread(MicroROSClient &client) {
    std::cout << "Started test thread" << std::endl;
    while (true) {
        int targetClassId;
        if (client.getDetectionsEnabled(targetClassId)) {
            // Some long running operation
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // This timestamp should be taken when the camera is triggered
            timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts)) {
                throw std::system_error(errno, std::generic_category(), "clock_gettime");
            }

            // Publish detection
            CameraDetection detection;
            detection.classId = "bootlegger";
            detection.score = 0.3;
            detection.pose.pose.position.x = 1;
            detection.pose.pose.position.y = 1;
            detection.pose.pose.position.z = 1;
            detection.pose.pose.orientation.w = 1;
            detection.pose.pose.orientation.x = 0;
            detection.pose.pose.orientation.y = 0;
            detection.pose.pose.orientation.z = 0;

            std::vector<CameraDetection> detections;
            detections.push_back(detection);
            client.reportDetections(ts, detections);
        }
        else {
            // Check every second to see if detections are enabled
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Expected interface name argument" << std::endl;
        return 1;
    }

    int ifIdx = if_nametoindex(argv[1]);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    auto &client = MicroROSClient::create(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, "talos");

    std::cout << "Waiting for agent to connect..." << std::endl;
    client.waitForAgent();

    std::thread t1(testThread, std::ref(client));

    std::cout << "Agent Connected!" << std::endl;
    client.run();

    return 0;
}
