#include "lccv.hpp"
#include "opencv2/opencv.hpp"
#include "tools/mySerial.h"

mySerial serialPort("/dev/ttyAMA4", 600);

void sendSerial() {
    serialPort.Send('\x00');
}

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}

int main(int argc, char *argv[]) {
    lccv::PiCamera camL, camR;

    createCamera(camL, 0);
    createCamera(camR, 1);
    camL.startVideo();
    camR.startVideo();

    for (int i = 0; i < 32; i++) {
        sendSerial();
    }


    sendSerial();

    cv::Mat leftIm, rightIm;

    if (!camL.getVideoFrame(leftIm, 99999999) || !camR.getVideoFrame(rightIm, 99999999)) {
        printf("Error while taking image");
        return -1;
    }

    cv::cvtColor(leftIm, leftIm, cv::COLOR_RGB2BGR);
    cv::cvtColor(rightIm, rightIm, cv::COLOR_RGB2BGR);

    cv::imwrite("/home/pi/Ex_Stereo_Imgs/leftIm.png", leftIm);
    cv::imwrite("/home/pi/Ex_Stereo_Imgs/rightIm.png", rightIm);

    exit(0);
}