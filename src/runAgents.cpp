#include "agents/cameraAgent.hpp"
#include "agents/orientationAgent.hpp"
#include "agents/stereoAgent.hpp"
#include "agents/yoloAgent.hpp"

#include <unistd.h>
#include <net/if.h>

int main(int argc, char *argv[]) {

    int ifIdx = if_nametoindex("talos");
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    MicroROSClient &client = MicroROSClient::create(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, "talos");

    client.waitForAgent();

    client.run();

    CameraAgent camAgent(std::ref(client));
    StereoAgent stereoAgent("/home/pi/StereoMaps.xml", "config", &camAgent);
    YoloAgent yoloAgent("/home/pi/BinZoomCrop_full_integer_quant_edgetpu.tflite", 10, .8, .8, &stereoAgent);
    OrientationAgent orientationAgent(std::ref(client), &yoloAgent);

    camAgent.startCapturing();
    stereoAgent.startStereo();
    yoloAgent.startDetecting();
    orientationAgent.startProducing();

    while (true) {
        usleep(1000);
    }

    return 0;
}
