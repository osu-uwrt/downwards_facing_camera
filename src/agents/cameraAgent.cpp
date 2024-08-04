#include <agents/cameraAgent.hpp>
#include <unistd.h>

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
}

void sendSerial(mySerial serial) {
    serial.Send('\x00');
}

CameraAgent::CameraAgent(MicroROSClient &client): running(true), capturing(false), client_(client) {
    createCamera(cam1, 0);
    createCamera(cam2, 1);

    cam1.startVideo();
    cam2.startVideo();

    sleep(1);

    for (int i = 0; i < 8; i++) {
        sendSerial(serialPort);
    }

    captureThread = std::thread(&CameraAgent::captureImages, this);
}

CameraAgent::~CameraAgent() {
    running = false;

    cam1.stopVideo();
    cam2.stopVideo();

    if (captureThread.joinable())
        captureThread.join();
}

void CameraAgent::startCapturing() {
    capturing = true;
}

void CameraAgent::stopCapturing() {
    capturing = false;
}

void CameraAgent::captureImages() {
    while (running) {
        if (capturing && imageQueue.size() < 1) {
            sendSerial(serialPort);
            timespec ts = client_.getAgentTime();
            cv::Mat images[2];
            if (!cam1.getVideoFrame(images[0], 1000) || !cam2.getVideoFrame(images[1], 1000)) {
                printf("Failed to grab frame\n");
                continue;
            }
            YoloDepth imageHandle;
            imageHandle.setImages(images[0], images[1]);
            imageHandle.setTimestamp(ts);
            imageQueue.push(imageHandle);
        } else {
            usleep(100);
        }
    }
}
