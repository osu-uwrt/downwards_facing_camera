#include "lccv.hpp"
#include <tools/mySerial.h>
#include <tools/DataUtils.hpp>

class CameraAgent {
public:
    TSQueue<YoloDepth> imageQueue;

    CameraAgent();
    ~CameraAgent();

    void startCapturing();
    void stopCapturing();


private:
    lccv::PiCamera cam1, cam2;
    mySerial serialPort = mySerial("/dev/ttyAMA4", 600);

    std::thread captureThread;

    bool running;
    bool capturing;

    void captureImages();

    void manageRequests();
};