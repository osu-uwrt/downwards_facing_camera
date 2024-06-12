#include "coral_yolo.hpp"
#include "stereoConfig.hpp"
#include "tools/mySerial.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <lccv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <unistd.h>
#include <vector>

mySerial serialPort("/dev/ttyAMA4", 600);
config stereoConfig;

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

void setupStereo(cv::Mat &leftStereoMap1, cv::Mat &leftStereoMap2, cv::Mat &rightStereoMap1, cv::Mat &rightStereoMap2,
                 cv::Mat &Q, cv::Ptr<cv::StereoBM> leftStereo, cv::Ptr<cv::StereoBM> rightStereo,
                 cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter) {
    // Set up remaps
    cv::FileStorage mapsFile("/home/pi/StereoMaps.xml", cv::FileStorage::READ);
    mapsFile["Left_Stereo_Map_x"] >> leftStereoMap1;
    mapsFile["Left_Stereo_Map_y"] >> leftStereoMap2;
    mapsFile["Right_Stereo_Map_x"] >> rightStereoMap1;
    mapsFile["Right_Stereo_Map_y"] >> rightStereoMap2;
    mapsFile["Q"] >> Q;
    mapsFile.release();

    stereoConfig = configFromFile();

    // TODO Read from YAML

    if (stereoConfig.SGBM) {
        leftStereo = cv::StereoSGBM::create(
            stereoConfig.minDisparity, stereoConfig.numDisparities, stereoConfig.blockSize, stereoConfig.P1,
            stereoConfig.P2, stereoConfig.disp12MaxDiff, stereoConfig.preFilterCap, stereoConfig.uniquenessRatio,
            stereoConfig.speckleWindowSize, stereoConfig.speckleRange);
    } else {
        leftStereo = cv::StereoBM::create(stereoConfig.numDisparities, stereoConfig.blockSize);
        leftStereo->setPreFilterType(stereoConfig.preFilterType);
        leftStereo->setPreFilterSize(stereoConfig.preFilterSize);
        leftStereo->setPreFilterCap(stereoConfig.preFilterCap);
        leftStereo->setTextureThreshold(stereoConfig.textureThreshold);
    }

    leftStereo->setMinDisparity(stereoConfig.minDisparity);
    leftStereo->setNumDisparities(stereoConfig.numDisparities);
    leftStereo->setBlockSize(stereoConfig.blockSize);
    // leftStereo->setUniquenessRatio(stereoConfig.uniquenessRatio);
    leftStereo->setSpeckleRange(stereoConfig.speckleRange);
    leftStereo->setSpeckleWindowSize(stereoConfig.speckleWindowSize);
    leftStereo->setDisp12MaxDiff(stereoConfig.disp12MaxDiff);
    // leftStereo

    rightStereo = cv::ximgproc::createRightMatcher(leftStereo);
    wslFilter = cv::ximgproc::createDisparityWLSFilter(rightStereo);
    wslFilter->setLambda(stereoConfig.lambda);
    wslFilter->setSigmaColor(stereoConfig.sigma);
}

void runStereo(cv::Mat &leftStereoMap1, cv::Mat &leftStereoMap2, cv::Mat &rightStereoMap1, cv::Mat &rightStereoMap2,
               cv::Ptr<cv::StereoBM> leftStereo, cv::Ptr<cv::StereoBM> rightStereo,
               cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter, cv::Mat &leftIm, cv::Mat &rightIm,
               cv::Mat &disparity) {
    cv::Mat leftGray, rightGray;

    cv::cvtColor(leftIm, leftGray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(rightIm, rightGray, cv::COLOR_RGB2GRAY);

    cv::Mat leftNice, rightNice, leftDisp, rightDisp;
    cv::remap(leftGray, leftNice, leftStereoMap1, leftStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(rightGray, rightNice, rightStereoMap1, rightStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

    leftStereo->compute(leftNice, rightNice, leftDisp);
    rightStereo->compute(rightNice, leftNice, rightDisp);
    leftDisp /= 16.0;
    rightDisp /= 16.0;
    wslFilter->filter(leftDisp, leftNice, disparity, rightDisp);
}

int main(int argc, char *argv) {
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
    cv::Mat Q;

    cv::Ptr<cv::StereoBM> leftStereo;
    cv::Ptr<cv::StereoBM> rightStereo;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter;

    setupStereo(Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, Q, leftStereo, rightStereo,
                wslFilter);

    cv::Mat disparity;

    cv::Mat leftIm, rightIm, cam_0_gray, cam_1_gray;
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
        if (!cam_0.getVideoFrame(leftIm, 1000) || !cam_1.getVideoFrame(rightIm, 1000)) {
            std::cout << "Timeout error " << counter << std::endl;
        }
        else {
            // Run stereo (Get depthmap)
            runStereo(Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, leftStereo, rightStereo,
                      wslFilter, leftIm, rightIm, disparity);

            cv::Mat depth_map = stereoConfig.baseline * stereoConfig.focalLen / disparity + stereoConfig.doffs + 1E-6;
            // Run YOLO

            // Get objects

            // Send Objects
        }
    }
}