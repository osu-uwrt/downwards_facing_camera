#include "calibration/pointGenerator.hpp"
#include "lccv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include "tools/CanmoreImageTransmitter.hpp"

#include "canmore/client_ids.h"

#include <filesystem>
#include <net/if.h>
#include <time.h>
#include <unistd.h>

std::vector<std::vector<cv::Point2f>> leftCornersVector, rightCornersVector;
std::vector<std::vector<cv::Point3f>> objPts;

std::string saveFoldername;

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
    int camId;
    bool useCan = false;

    if (argc >= 2) {
        if (std::stoi(argv[1]) == 1) {
            printf("Displaying image over CAN\n");
            useCan = true;
        }
        else {
            printf("Displaying image over X Server\n");
        }
    }
    else {
        printf("Displaying image over X Server\n");
    }
    CanmoreImageTransmitter *imageTx;

    if (useCan) {
        int ifIdx = if_nametoindex("can0");
        if (!ifIdx) {
            throw std::system_error(errno, std::generic_category(), "if_nametoindex");
        }
        imageTx = new CanmoreImageTransmitter(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, 190, 20, true);
    }

    createCamera(camL, 0);
    createCamera(camR, 1);
    camL.startVideo();
    camR.startVideo();

    for (int i = 0; i < 16; i++) {
        sendSerial();
    }

    char ch = 0;

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

    cv::Mat leftIm, rightIm, leftVis, rightVis, vis;

    cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::ArucoDetector detector(dict, params);

    cv::Size patternSize_(6, 8);

    cv::Size imageSize(camL.options->video_width, camL.options->video_height);

    std::vector<CalibrationPose> points = generateSterPoints(imageSize);

    // CalibrationPose tagLoc = points[std::stoi(argv[1])];

    int currentPose = 0;

    while (currentPose < points.size()) {
        sendSerial();
        // printf("NOT PAST\n");
        if (!camL.getVideoFrame(leftIm, 99999999) || !camR.getVideoFrame(rightIm, 99999999)) {
            std::cout << "Timeout error " << std::endl;
        }

        // printf("PAST\n");

        cv::cvtColor(leftIm, leftVis, cv::COLOR_RGB2BGR);
        cv::cvtColor(rightIm, rightVis, cv::COLOR_RGB2BGR);

        CalibrationPose leftTagLoc = points[currentPose + 1];
        CalibrationPose rightTagLoc = points[currentPose];

        cv::circle(leftVis, leftTagLoc.tag0Pos, 5, cv::Scalar(127, 255, 255), 5);
        cv::circle(leftVis, leftTagLoc.tag1Pos, 5, cv::Scalar(0, 0, 255), 5);

        cv::circle(rightVis, rightTagLoc.tag0Pos, 5, cv::Scalar(127, 255, 255), 5);
        cv::circle(rightVis, rightTagLoc.tag1Pos, 5, cv::Scalar(0, 0, 255), 5);

        std::vector<int> leftMarkerIds, rightMarkerIds;
        std::vector<std::vector<cv::Point2f>> leftMarkerCorners, leftRejectedCandidates, rightMarkerCorners,
            rightRejectedCandidates;
        detector.detectMarkers(leftIm, leftMarkerCorners, leftMarkerIds, leftRejectedCandidates);

        cv::aruco::drawDetectedMarkers(leftVis, leftMarkerCorners, leftMarkerIds);

        detector.detectMarkers(rightIm, rightMarkerCorners, rightMarkerIds, rightRejectedCandidates);

        cv::aruco::drawDetectedMarkers(rightVis, rightMarkerCorners, rightMarkerIds);

        std::vector<cv::Point2f> leftTag0Corners, leftTag1Corners, rightTag0Corners, rightTag1Corners;
        for (int i = 0; i < leftMarkerIds.size(); i++) {
            if (leftMarkerIds[i] == 0) {
                leftTag0Corners = leftMarkerCorners[i];
            }
            else if (leftMarkerIds[i] == 1) {
                leftTag1Corners = leftMarkerCorners[i];
            }
        }

        for (int i = 0; i < rightMarkerIds.size(); i++) {
            if (rightMarkerIds[i] == 0) {
                rightTag0Corners = rightMarkerCorners[i];
            }
            else if (rightMarkerIds[i] == 1) {
                rightTag1Corners = rightMarkerCorners[i];
            }
        }

        if (leftTag0Corners.size() > 0 && leftTag1Corners.size() > 0 && rightTag0Corners.size() > 0 &&
            rightTag1Corners.size() > 0) {
            cv::Mat mean;
            cv::reduce(leftTag0Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f leftTag0Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));
            cv::reduce(leftTag1Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f leftTag1Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));

            cv::reduce(rightTag0Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f rightTag0Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));
            cv::reduce(rightTag1Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f rightTag1Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));

            double leftTag0dist = cv::norm(leftTagLoc.tag0Pos - leftTag0Pos);
            double leftTag1dist = cv::norm(leftTagLoc.tag1Pos - leftTag1Pos);

            double rightTag0dist = cv::norm(rightTagLoc.tag0Pos - rightTag0Pos);
            double rightTag1dist = cv::norm(rightTagLoc.tag1Pos - rightTag1Pos);

            if (leftTag0dist < 20 && leftTag1dist < 20) {
                cv::Mat leftGrayIm, rightGrayIm;
                cv::cvtColor(leftIm, leftGrayIm, cv::COLOR_RGB2GRAY);
                cv::cvtColor(rightIm, rightGrayIm, cv::COLOR_RGB2GRAY);

                std::vector<cv::Point2f> leftCornerPts, rightCornerPts;
                bool leftFound = cv::findChessboardCornersSB(leftGrayIm, patternSize_, leftCornerPts);
                bool rightFound = cv::findChessboardCornersSB(rightGrayIm, patternSize_, rightCornerPts);

                if (leftFound && rightFound) {
                    cv::drawChessboardCorners(leftVis, patternSize_, leftCornerPts, leftFound);
                    cv::drawChessboardCorners(rightVis, patternSize_, rightCornerPts, rightFound);

                    if (ch == 'c') {
                        objPts.push_back(objp);
                        leftCornersVector.push_back(leftCornerPts);
                        rightCornersVector.push_back(rightCornerPts);

                        currentPose += 2;

                        cv::imwrite(saveFoldername + "Left_" + std::to_string(i) + ".png", leftIm);
                        cv::imwrite(saveFoldername + "Right_" + std::to_string(i) + ".png", rightIm);
                        ch = 0;
                    }
                }
            }

            // printf("Tag 0 pos: %f, %f\n", tag0Pos.x, tag0Pos.y);
            // printf("Tag 1 pos: %f, %f\n", tag1Pos.x, tag1Pos.y);
        }

        // printf("%d markers detected\n", markerIds.size());
        cv::hconcat(leftVis, rightVis, vis);

        if (useCan) {
            imageTx->transmitImage(vis);
            usleep(200000);
        }
        else {
            cv::imshow("Image", vis);
            ch = cv::waitKey(1) & 255;
        }

        if (ch == 'q') {
            break;
        }

        // if (ch == 'c') {
        //     currentPose++;
        // }
    }

    cv::Mat camMat, distCoeffs, R, T;

    bool calibrated = cv::calibrateCamera(objPts, imgPts, imageSize, camMat, distCoeffs, R, T);
    if (calibrated) {
        printf("Calibrated, finding optimal\n");
        cv::Mat newMat = cv::getOptimalNewCameraMatrix(camMat, distCoeffs, imageSize, 1, imageSize);
        printf("Writing to file\n");
        std::string saveLoc = "/home/pi/Cam" + std::to_string(camId) + "Intr.xml";
        cv::FileStorage cvFile(saveLoc, cv::FileStorage::WRITE);
        cvFile.write("Matrix", newMat);
        cvFile.write("DistCoeffs", distCoeffs);
        cvFile.release();
    }
    else {
        printf("Failed to calibrate\n");
    }

    cv::destroyAllWindows();
    exit(0);
    camL.stopVideo();
}