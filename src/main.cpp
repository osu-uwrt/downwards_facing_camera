#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <unistd.h>
#include <time.h>
#include "tools/mySerial.h"

mySerial serialPort("/dev/ttyAMA0", 600);

void createCamera(lccv::PiCamera &camera_, int id)
{
    camera_.options->camera = id;
    camera_.options->video_width = 1456;
    camera_.options->video_height = 1088;
    camera_.options->framerate = 60;
}

void sendSerial()
{
    serialPort.Send('\x00');
}

int main()
{

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 123);
    stereo->setMinDisparity(0);
    cv::Mat disparity;

    cv::namedWindow("Video", cv::WINDOW_NORMAL);
    cv::namedWindow("Video1", cv::WINDOW_NORMAL);

    cv::Mat cam_0_im, cam_1_im, cam_0_gray, cam_1_gray;
    lccv::PiCamera cam_0, cam_1;
    createCamera(cam_0, 0);
    createCamera(cam_1, 1);

    cam_0.startVideo();
    std::cout << "Camera 0 Started" << std::endl;
    cam_1.startVideo();
    std::cout << "Camera 1 Started" << std::endl;
    int ch = 0;
    int counter = 0;
    sleep(1); // Let libcamera start (And why do you want it so fast smh)

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++)
    {
        sendSerial();
    }

    time_t start = time(0);
    while (ch != 27)
    {
        sendSerial();
        if (!cam_0.getVideoFrame(cam_0_im, 99999999) || !cam_1.getVideoFrame(cam_1_im, 99999999))
        {
            std::cout << "Timeout error " << counter << std::endl;
        }
        else
        {
            cv::cvtColor(cam_0_im, cam_0_gray, cv::COLOR_RGB2GRAY);
            cv::cvtColor(cam_1_im, cam_1_gray, cv::COLOR_RGB2GRAY);
            stereo->compute(cam_0_gray, cam_1_gray, disparity);
            disparity.convertTo(disparity, CV_32F, 1.0);
            disparity = (disparity/16.0f - 0.0f) / 16;
            cv::imshow("disparity", disparity);
            cv::imshow("Video", cam_0_gray);
            cv::imshow("Video1", cam_1_gray);
            ch = cv::waitKey(10);
            counter++;
        }
        if ((time(0) - start) >= 1)
        {
            printf("FPS: %d\n", counter);
            start = time(0);
            counter = 0;
            double max, min;
        }
    }
    cam_0.stopVideo();
    cam_1.stopVideo();
    cv::destroyWindow("Video");
}
