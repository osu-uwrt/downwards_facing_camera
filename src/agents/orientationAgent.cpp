#include "agents/orientationAgent.hpp"

#include <iostream>
#include <net/if.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

OrientationAgent::OrientationAgent(char *id, YoloAgent *yoAgent) {
    yoloAgent = yoAgent;

    int ifIdx = if_nametoindex(id);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    client = MicroROSClient::create(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, "talos");

    client.waitForAgent();

    running = true;

    producingThread = std::thread(&OrientationAgent::produce, this);
}

OrientationAgent::~OrientationAgent() {
    if (producingThread.joinable())
        producingThread.join();
}

void OrientationAgent::startProducing() {
    producing = true;
}

void OrientationAgent::stopProducing() {
    producing = false;
}

void OrientationAgent::produce() {
    while (running) {
        if (producing) {
            YoloDepth inputs;
            try {
                inputs = yoloAgent->output.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            // Do the orientation things with inputs
        }
    }
}