#ifndef CAMERAAGENT
#define CAMERAAGENT

#include "lccv.hpp"
#include <tools/mySerial.h>
#include <tools/DataUtils.hpp>

#include "tools/DFCMicroROSClient.hpp"

class CameraAgent {
public:
    TSQueue<YoloDepth> imageQueue;

    CameraAgent(MicroROSClient &client);
    ~CameraAgent();

    void startCapturing();
    void stopCapturing();


private:
    lccv::PiCamera cam1, cam2;
    mySerial serialPort = mySerial("/dev/ttyAMA4", 600);

    MicroROSClient &client_;

    std::thread captureThread;

    bool running;
    bool capturing;

    void captureImages();
};

#endif
