#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "tools/mySerial.h"

mySerial serialPort("/dev/ttyAMA0", 450);

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width=1456;
    camera_.options->video_height=1088;
    camera_.options->framerate=60;
}

void sendSerial() {
    serialPort.Send('\x00');
}

int main()
{
    cv::namedWindow("Video",cv::WINDOW_NORMAL);
    cv::namedWindow("Video1", cv::WINDOW_NORMAL);

    cv::Mat cam_0_im, cam_1_im;
    lccv::PiCamera cam_0, cam_1;
    createCamera(cam_0, 0);
    createCamera(cam_1, 1);

    cam_0.startVideo();
    std::cout << "Camera 0 Started" << std::endl;
    cam_1.startVideo();
    std::cout << "Camera 1 Started" << std::endl;
    int ch=0;
    int counter = 0;
    sleep(1);

    // Need to fill up buffer (fairly sure why we need to do this)
    for (int i = 0; i < 8; i++) {
        sendSerial();
    }

    while(ch!=27){
        sendSerial();
        if(!cam_0.getVideoFrame(cam_0_im,99999999) || !cam_1.getVideoFrame(cam_1_im, 99999999)){
            std::cout<<"Timeout error " << counter <<std::endl;
            counter++;
        }
        else{
            cv::imshow("Video",cam_0_im);
            cv::imshow("Video1",cam_1_im);
            ch=cv::waitKey(10);
            counter = 0;
        }
    }
    cam_0.stopVideo();
    cam_1.stopVideo();
    cv::destroyWindow("Video");
}
