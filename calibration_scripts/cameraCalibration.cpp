#include "cameraCalibration.hpp"

#include "lccv.hpp"
#include "tools/CanmoreImageTransmitter.hpp"

#include "canmore/client_ids.h"

#include <chrono>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

std::vector<std::vector<cv::Point3f>> objPts;
std::vector<std::vector<cv::Point2f>> imgPts;

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
    lccv::PiCamera camera;
    int camId;
    bool useCan;

    if (argc >= 2) {
        camId = std::stoi(argv[1]);
        if (camId == 0 || camId == 1) {
            printf("Calibrating camera: %d\n", camId);
        }
        else {
            printf("Invalid arguement: %d\n", camId);
            return (0);
        }
        if (argc >= 3) {
            if (std::stoi(argv[2]) == 1) {
                printf("Displaying image over CAN\n");
                useCan = true;
            }
            else {
                printf("Displaying image over X Server\n");
                useCan = false;
            }
        }
    }
    else {
        camId = 0;
        useCan = false;
        printf("No camera specified, defaulting to 0\n");
        printf("Displaying image over X Server\n");
    }

    if (useCan) {
        int ifIdx = if_nametoindex(argv[1]);
        if (!ifIdx) {
            throw std::system_error(errno, std::generic_category(), "if_nametoindex");
        }
        CanmoreImageTransmitter imageTx(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA);
    }

    createCamera(camera, camId);
    camera.startVideo();

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++) {
        sendSerial();
    }

    int keyPress = 0;

    int i = 0;
    int currentNum;
    cv::Mat displayImage, image;

    std::vector<cv::Point3f> objp;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (i < totalImages) {
        currentNum = 1;
        // while (currentNum > -1) {
        // Press Q to advance
        while (keyPress != 113) {
            sendSerial();
            if (!camera.getVideoFrame(image, 99999999)) {
                printf("Couldn't grab frame\n");
                continue;
            }
            cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
            image.copyTo(displayImage);
            cv::putText(displayImage, std::to_string(currentNum), cv::Point2d(60, 0), cv::FONT_HERSHEY_PLAIN, 4,
                        cv::Scalar(0, 255, 0), 5);

            // For timebased images if you're doing it yourself
            // std::string progressText = std::to_string(i + 1) + '/' + std::to_string(totalImages);
            // cv::putText(displayImage, progressText, cv::Point2d(image.cols - 200, 60), cv::FONT_HERSHEY_PLAIN, 4,
            //             cv::Scalar(0, 255, 0), 5);

            if (useCan) {
            }
            else {
                cv::imshow("Calibration", displayImage);
                keyPress = cv::waitKey(1) & 255;
            }

            // For timebased images if you're doing it yourself
            // if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
            //         .count() >= 1) {
            //     currentNum -= 1;
            //     start = std::chrono::high_resolution_clock::now();
            // }
        }
        std::vector<cv::Point2f> cornerPts;
        bool found = cv::findChessboardCornersSB(image, patternSize, cornerPts);
        if (found) {
            cv::drawChessboardCorners(displayImage, patternSize, cornerPts, found);
            start = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start)
                       .count() < 1) {
                // Just show the checkerboard
                if (useCan) {
                }
                else {
                    cv::imshow("Calibration", displayImage);
                    keyPress = cv::waitKey(1) & 255;
                }
            }
            objPts.push_back(objp);
            imgPts.push_back(cornerPts);
            i++;
        }
        else {
            printf("Can't not find chessboard\n");
        }
    }

    printf("Calibration\n");

    cv::Mat camMat, distCoeffs, R, T;

    bool calibrated = cv::calibrateCamera(objPts, imgPts, imageSize, camMat, distCoeffs, R, T, cv::CALIB_USE_LU);
    if (calibrated) {
        printf("Calibrated, finding optimal\n");
        cv::Mat newMat = cv::getOptimalNewCameraMatrix(camMat, distCoeffs, imageSize, 1, imageSize);
        printf("Writing to file\n");
        std::string saveLoc = "/home/pi/Cam" + std::to_string(camId) + "Intr.xml";
        cv::FileStorage cvFile(saveLoc, cv::FileStorage::WRITE);
        cvFile.write("Matrix", newMat);
        cvFile.write("DistCoeffs", distCoeffs);
        // cvFile.release();
    }
    else {
        printf("Failed to calibrate\n");
    }
    return 0;
}
