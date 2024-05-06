
#include "calibration/stereoCalibration.hpp"

#include "lccv.hpp"
#include "tools/CanmoreImageTransmitter.hpp"
#include "canmore/client_ids.h"

#include <chrono>
#include <net/if.h>
#include <filesystem>
#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

void sendSerial() {
    serialPort.Send('\x00');
}

void createCamera(lccv::PiCamera &camera_, int id) {
    camera_.options->camera = id;
    camera_.options->video_width = imageSize.width;
    camera_.options->video_height = imageSize.height;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}

int main(int argc, char *argv[]) {
    lccv::PiCamera camL, camR;

    createCamera(camL, 0);
    createCamera(camR, 1);

    camL.startVideo();
    camR.startVideo();

    bool useCan = false;

    if (argc >= 2) {
        useCan = std::stoi(argv[1]);
        if (useCan == 1) {
            printf("Displaying image over can\n");
            useCan = true;
        }
        else {
            printf("Displaying image over X server\n");
        }
    } else {
            printf("Displaying image over X server\n");
    }

    CanmoreImageTransmitter *imageTx;

    if (useCan) {
        int ifIdx = if_nametoindex("can0");
        if (!ifIdx) {
            throw std::system_error(errno, std::generic_category(), "if_nametoindex");
        }
        imageTx = new CanmoreImageTransmitter(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, 190, 20, true);
    }

    std::string saveFoldername;

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++) {
        sendSerial();
    }

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

    std::stringstream stream;
    std::time_t current_time = std::time(nullptr);
    std::tm tm = *std::localtime(&current_time);
    stream << std::put_time(&tm, "%Y%m%d%H%M");
    saveFoldername = "/home/pi/CalibrationImgs/" + stream.str() + "_StereoImgs/";
    std::filesystem::create_directories(saveFoldername.c_str());

    int keyPress = 0;

    while (i < totalImages) {
        int currentNum = 1;
        auto start = std::chrono::high_resolution_clock::now();
        //while (keyPress != 'c') {
        while (currentNum > -1) {
            sendSerial();
            if (!camL.getVideoFrame(leftIm, 99999999) || !camR.getVideoFrame(rightIm, 99999999)) {
                printf("Cannot grab video frame\n");
            }
            cv::cvtColor(leftIm, leftIm, cv::COLOR_RGB2GRAY);
            cv::cvtColor(rightIm, rightIm, cv::COLOR_RGB2GRAY);
            cv::hconcat(leftIm, rightIm, vis);
            cv::putText(vis, std::to_string(currentNum), cv::Point2d(60, 0), cv::FONT_HERSHEY_PLAIN, 4,
                        cv::Scalar(0, 255, 0), 5);
            std::string displayText = std::to_string(i + 1) + "/" + std::to_string(totalImages);
            cv::putText(vis, displayText, cv::Point(60, leftIm.rows - 200), cv::FONT_HERSHEY_PLAIN, 4,
                        cv::Scalar(0, 255, 0), 5);
	    cv::cvtColor(vis, vis, cv::COLOR_GRAY2RGB);
            // cv::resize(vis, vis, cv::Size(vis.cols / 1.75, vis.rows / 1.75));
            if (useCan) {
                imageTx->transmitImage(vis);
                usleep(200000);
                keyPress = imageTx->getKeypress();
            } else {
                cv::imshow("Calibration", vis);
                keyPress = cv::waitKey(1) & 255;
            }
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                    .count() >= 1) {
                currentNum -= 1;
                start = std::chrono::high_resolution_clock::now();
            }
        }
        //char *filename = new char[100];
        //sprintf(filename, "/home/pi/202404161747_StereoImgs/Left_%i.png", i);
        //leftIm = cv::imread(filename);
        //filename = new char[100];
        //sprintf(filename, "/home/pi/202404161747_StereoImgs/Right_%i.png", i);
        //rightIm = cv::imread(filename);

        std::vector<cv::Point2f> leftCorners, rightCorners;
        bool leftFound = cv::findChessboardCornersSB(leftIm, patternSize, leftCorners);
        bool rightFound = cv::findChessboardCornersSB(rightIm, patternSize, rightCorners);
        if (leftFound && rightFound) {
            objPts.push_back(objp);
            leftCornersVector.push_back(leftCorners);
            rightCornersVector.push_back(rightCorners);
            cv::imwrite(saveFoldername + "Left_" + std::to_string(i) + ".png", leftIm);
            cv::imwrite(saveFoldername + "Right_" + std::to_string(i) + ".png", rightIm);
            cv::drawChessboardCorners(leftIm, patternSize, leftCorners, leftFound);
            cv::drawChessboardCorners(rightIm, patternSize, rightCorners, rightFound);
            cv::hconcat(leftIm, rightIm, vis);
            start = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                       .count() < 0) {
                // Just show the checkerboard
                cv::imshow("Calibration", vis);
                cv::waitKey(1);
            }
            i++;
        }
        else {
            if (leftFound) {
                printf("Could not find right chessboard\n");
            }
            else if (rightFound) {
                printf("Could not find left chessboard\n");
            }
            else {
                printf("Could not find any chessboard\n");
            }
        }
        keyPress = 0;
    }

    // camL.stopVideo();
    // camR.stopVideo();
    
    cv::FileStorage leftIntrFile("/home/pi/Cam0Intr.xml", cv::FileStorage::READ);
    cv::FileStorage rightIntrFile("/home/pi/Cam1Intr.xml", cv::FileStorage::READ);

    cv::Mat camMatL, camMatR, camDCL, camDCR;
    leftIntrFile["Matrix"] >> camMatL;
    leftIntrFile["DistCoeffs"] >> camDCL;
    rightIntrFile["Matrix"] >> camMatR;
    rightIntrFile["DistCoeffs"] >> camDCR;
    leftIntrFile.release();
    rightIntrFile.release();

    int flags = 0 | cv::CALIB_FIX_INTRINSIC;

    cv::Mat R, T, E, F;
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    double retS = cv::stereoCalibrate(objPts, leftCornersVector, rightCornersVector, camMatL, camDCL, camMatR, camDCR,
                                      imageSize, R, T, E, F, flags, criteria);
    if (retS) {
        printf("Stereo Calibrated\n");
    }
    else {
        printf("Stereo Calibration Failed\n");
    }

    cv::Mat rectL, rectR, projL, projR, Q, roiL, roiR;
    cv::stereoRectify(camMatL, camDCL, camMatR, camDCR, leftIm.size(), R, T, rectL, rectR, projL, projR, Q, 1);
    cv::Mat stereoMapLX, stereoMapLY, stereoMapRX, stereoMapRY;
    cv::initUndistortRectifyMap(camMatL, camDCL, rectL, projL, imageSize, CV_16SC2, stereoMapLX, stereoMapLY);
    cv::initUndistortRectifyMap(camMatR, camDCR, rectR, projR, imageSize, CV_16SC2, stereoMapRX, stereoMapRY);
    // printf("%d\n", stereoMapRY.type());

    cv::FileStorage cvFile = cv::FileStorage("/home/pi/StereoMaps.xml", cv::FileStorage::WRITE);
    cvFile.write("Left_Stereo_Map_x", stereoMapLX);
    cvFile.write("Left_Stereo_Map_y", stereoMapLY);
    cvFile.write("Right_Stereo_Map_x", stereoMapRX);
    cvFile.write("Right_Stereo_Map_y", stereoMapRY);
    cvFile.release();
}
