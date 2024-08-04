#include "agents/cameraAgent.hpp"
#include "agents/orientationAgent.hpp"
#include "agents/yoloAgent.hpp"

#include <unistd.h>
#include <net/if.h>

void runClient(MicroROSClient &client) {
    client.run();
}

int main(int argc, char *argv[]) {

    int ifIdx = if_nametoindex("can0");
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    MicroROSClient &client = MicroROSClient::create(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, "talos");

    printf("Waiting for agent (client id %d)\n", CANMORE_CLIENT_ID_DOWNWARDS_CAMERA);
    client.waitForAgent();

    printf("Client connected\n");
    std::thread newThread = std::thread(&runClient, std::ref(client));

    CameraAgent camAgent(std::ref(client));
    YoloAgent yoloAgent("/home/pi/robosub_2024_1_full_integer_quant_edgetpu.tflite", 10, .8, .8, &camAgent);
    OrientationAgent orientationAgent(std::ref(client), &yoloAgent);

    camAgent.startCapturing();
    yoloAgent.startDetecting();
    orientationAgent.startProducing();

    while (true) {
        usleep(1000);
    }

    newThread.join();

    return 0;
}
