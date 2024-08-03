#include "lccv.hpp"
#include "tools/mySerial.h"

#include "opencv2/highgui.hpp"

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
}

void sendSerial(mySerial serial) {
    serial.Send('\x00');
}

int main(int argc, char *argv[]) {
    lccv::PiCamera cam1, cam2;
    mySerial serialPort = mySerial("/dev/ttyAMA4", 600);

    createCamera(cam1, 0);
    createCamera(cam2, 1);

    cam1.startVideo();
    cam2.startVideo();

    for (int i = 0; i < 16; i++) {
        sendSerial(serialPort);
    }

    int ch = 0;

    while (ch != 27) {
        cv::Mat im1, im2;
        cv::Mat vis = cv::Mat::zeros(cv::Size(640, 360), CV_8U);
        sendSerial(serialPort);
        if (!cam1.getVideoFrame(im1, 1000) || !cam2.getVideoFrame(im2, 1000)) {
            printf("Fail\n");
            // std::cout << "Timeout error " << counter << std::endl;
        }
        else {
            cv::hconcat(im1, im2, vis);

            printf("Displaying image\n");
        }

        cv::imshow("Test", vis);
        ch = cv::waitKey(1);
    }

    cam1.stopVideo();
    cam2.stopVideo();
}