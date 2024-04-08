#include "lccv.hpp"
#include "tools/mySerial.h"

#include <chrono>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

int totalImages = 41;

mySerial serialPort("/dev/ttyAMA0", 600);

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

int main(int argc, char* argv[]) {
    lccv::PiCamera camL, camR;

    createCamera(camL, 0);
    createCamera(camR, 1);

    camL.startVideo();
    camR.startVideo();

    int i = 0;
    cv::Mat leftIm, rightIm, vis;
    std::vector<std::vector<cv::Point2f>> leftCornersVector, rightCornersVector;
    std::vector<std::vector<cv::Point3f>> objPts;

    std::vector<cv::Point3f> objp;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    while (i < totalImages) {
        int currentNum = 2;
        auto start = std::chrono::high_resolution_clock::now();
        while (currentNum > -1) {
            sendSerial();
            if (!camL.getVideoFrame(leftIm, 99999999) || !camR.getVideoFrame(rightIm, 99999999)) {
                printf("Cannot grab video frame\n");
            }
            cv::cvtColor(leftIm, leftIm, cv::COLOR_RGB2GRAY);
            cv::cvtColor(rightIm, rightIm, cv::COLOR_RGB2GRAY);
            cv::hconcat(leftIm, rightIm, vis);
            cv::putText(vis, std::to_string(currentNum), cv::Point2d(60, 0), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 0), 5);
            std::string displayText = std::to_string(i + 1) + "/" + std::to_string(totalImages);
            cv::putText(vis, displayText, cv::Point(60, leftIm.rows - 200), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0, 255, 0), 5);
            cv::resize(vis, vis, cv::Size(vis.rows / 1.75, vis.cols / 1.75));
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                    .count() >= 1) {
                currentNum -= 1;
                start = std::chrono::high_resolution_clock::now();
            }
        }
        std::vector<cv::Point2f> leftCorners, rightCorners;
        bool leftFound = cv::findChessboardCornersSB(leftIm, cv::Size(6, 8), leftCorners);
        bool rightFound = cv::findChessboardCornersSB(rightIm, cv::Size(6, 8), rightCorners);
        if (leftFound && rightFound) {
            objPts.push_back(objp);
            leftCornersVector.push_back(leftCorners);
            rightCornersVector.push_back(rightCorners);
            cv::drawChessboardCorners(leftIm, cv::Size(6, 8), leftCorners, leftFound);
            cv::drawChessboardCorners(rightIm, cv::Size(6, 8), rightCorners, rightFound);
            start = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                       .count() < 2) {
                // Just show the checkerboard
            }
            i++;
        } else {
            if (leftFound) {
                printf("Could not find right chessboard\n");
            } else if (rightFound) {
                printf("Could not find left chessboard\n");
            } else {
                printf("Could not find any chessboard\n");
            }
        }
    }
    cv::FileStorage leftIntrFile("/home/pi/Cam0Intr.xml", cv::FileStorage::READ);
    cv::FileStorage rightIntrFile("/home/pi/Cam1Intr.xml", cv::FileStorage::READ);

    cv::Mat camMatL, camMatR, camDCL, camDCR;
    leftIntrFile["Matrix"] >> camMatL;
    leftIntrFile["DistCoeffs"] >> camDCL;
    rightIntrFile["Matrix"] >> camMatR;
    rightIntrFile["DistCoeffs"] >> camDCR;

    int flags = 0 | cv::CALIB_FIX_INTRINSIC;

    cv::Mat R, T, E, F;
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    double retS = cv::stereoCalibrate(objPts, leftCornersVector, rightCornersVector, camMatL, camDCL, camMatR, camDCR, cv::Size(640, 360), R, T, E, F, flags, criteria);
    if (retS) {
        printf("Stereo Calibrated\n");
    } else {
        printf("Stereo Calibration Failed\n");
    }

    cv::Mat rectL, rectR, projL, projR, Q, roiL, roiR;
    cv::stereoRectify(camMatL, camDCL, camMatR, camDCR, leftIm.size(), R, T, rectL, rectR, projL, projR, Q, 1);
    cv::Mat stereoMapLX, stereoMapLY, stereoMapRX, stereoMapRY;
    cv::initUndistortRectifyMap(camMatL, camDCL, rectL, projL, leftIm.size(), CV_16SC2, stereoMapLX, stereoMapLY);
    cv::initUndistortRectifyMap(camMatR, camDCR, rectR, projR, rightIm.size(), CV_16SC2, stereoMapRX, stereoMapRY);

    cv::FileStorage cvFile = cv::FileStorage("/home/pi/StereoMaps.xml", cv::FileStorage::WRITE);
    cvFile.write("Left_Stereo_Map_x",stereoMapLX);
    cvFile.write("Left_Stereo_Map_y",stereoMapLY);
    cvFile.write("Right_Stereo_Map_x",stereoMapRX);
    cvFile.write("Right_Stereo_Map_y",stereoMapRX);
    cvFile.release();
}