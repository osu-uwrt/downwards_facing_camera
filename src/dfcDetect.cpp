#include "coral_yolo.hpp"
#include "tools/mySerial.h"

#include <lccv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <time.h>
#include <unistd.h>

mySerial serialPort("/dev/ttyAMA4", 600);

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}

void setupStereo(cv::Mat &leftStereoMap1, cv::Mat &leftStereoMap2, cv::Mat &rightStereoMap1, cv::Mat &rightStereoMap2,
                 cv::Ptr<cv::StereoBM> leftStereo, cv::Ptr<cv::StereoBM> rightStereo,
                 cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter) {
    // Set up remaps
    cv::FileStorage mapsFile("/home/pi/StereoMaps.xml", cv::FileStorage::READ);
    mapsFile["Left_Stereo_Map_x"] >> leftStereoMap1;
    mapsFile["Left_Stereo_Map_y"] >> leftStereoMap2;
    mapsFile["Right_Stereo_Map_x"] >> rightStereoMap1;
    mapsFile["Right_Stereo_Map_y"] >> rightStereoMap2;
    mapsFile.release();

    leftStereo->setMinDisparity(0);
    leftStereo->setNumDisparities(128);
    leftStereo->setBlockSize(55);  // 15
    leftStereo->setPreFilterType(1);
    leftStereo->setPreFilterSize(7);
    leftStereo->setPreFilterCap(62);
    leftStereo->setTextureThreshold(100);
    leftStereo->setUniquenessRatio(0);
    leftStereo->setSpeckleRange(100);
    leftStereo->setSpeckleWindowSize(3);
    leftStereo->setDisp12MaxDiff(-1);

    rightStereo = cv::ximgproc::createRightMatcher(leftStereo);
    wslFilter = cv::ximgproc::createDisparityWLSFilter(rightStereo);
    wslFilter->setLambda(12000);
    wslFilter->setSigmaColor(2.0);
}

void runStereo(cv::Mat &leftStereoMap1, cv::Mat &leftStereoMap2, cv::Mat &rightStereoMap1, cv::Mat &rightStereoMap2,
               cv::Ptr<cv::StereoBM> leftStereo, cv::Ptr<cv::StereoBM> rightStereo,
               cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter, cv::Mat &leftIm, cv::Mat &rightIm,
               cv::Mat &disparity) {
    cv::Mat leftGray, rightGray;

    cv::cvtColor(leftIm, leftGray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(rightIm, rightGray, cv::COLOR_RGB2GRAY);
    // cv::imshow("Video", cam_0_im);
    // cv::imshow("Video1", cam_1_im);

    cv::Mat leftNice, rightNice, leftDisp, rightDisp;
    cv::remap(leftGray, leftNice, leftStereoMap1, leftStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(rightGray, rightNice, rightStereoMap1, rightStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    // printf("%d, %d\n", leftNice.size().height, leftNice.size().width);

    leftStereo->compute(leftNice, rightNice, leftDisp);
    rightStereo->compute(rightNice, leftNice, rightDisp);
    wslFilter->filter(leftDisp, leftNice, disparity, rightDisp);
}

void sendSerial() {
    serialPort.Send('\x00');
}

int main() {
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::Ptr<cv::StereoBM> leftStereo;
    cv::Ptr<cv::StereoBM> rightStereo;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter;

    setupStereo(Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, leftStereo, rightStereo,
                wslFilter);

    cv::Mat disparity;

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
    sleep(1);  // Let libcamera start (And why do you want it so fast smh)

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++) {
        sendSerial();
    }

    while (true) {
        sendSerial();
        if (!cam_0.getVideoFrame(cam_0_im, 99999999) || !cam_1.getVideoFrame(cam_1_im, 99999999)) {
            std::cout << "Timeout error " << counter << std::endl;
        }
        else {
            runStereo(Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, leftStereo, rightStereo,
                      wslFilter, cam_0_im, cam_1_im, disparity);
            disparity.convertTo(disparity, CV_32F, 1.0);

            // Now do the things
        }
    }
}