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
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
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
        // printf("%d", points.size());

        cv::cvtColor(leftIm, leftVis, cv::COLOR_RGB2BGR);
        cv::cvtColor(rightIm, rightVis, cv::COLOR_RGB2BGR);

        CalibrationPose leftTagLoc = points[currentPose + 1];
        CalibrationPose rightTagLoc = points[currentPose];

        // printf("Drawing circles\n");

        cv::circle(leftVis, leftTagLoc.tag0Pos, 5, cv::Scalar(127, 255, 255), 5);
        cv::circle(leftVis, leftTagLoc.tag1Pos, 5, cv::Scalar(0, 0, 255), 5);

        cv::circle(rightVis, rightTagLoc.tag0Pos, 5, cv::Scalar(127, 255, 255), 5);
        cv::circle(rightVis, rightTagLoc.tag1Pos, 5, cv::Scalar(0, 0, 255), 5);

        std::vector<int> leftMarkerIds, rightMarkerIds;
        std::vector<std::vector<cv::Point2f>> leftMarkerCorners, leftRejectedCandidates, rightMarkerCorners,
            rightRejectedCandidates;

        // printf("Detecting Markers\n");
        detector.detectMarkers(leftIm, leftMarkerCorners, leftMarkerIds, leftRejectedCandidates);

        cv::aruco::drawDetectedMarkers(leftVis, leftMarkerCorners, leftMarkerIds);

        detector.detectMarkers(rightIm, rightMarkerCorners, rightMarkerIds, rightRejectedCandidates);

        cv::aruco::drawDetectedMarkers(rightVis, rightMarkerCorners, rightMarkerIds);

        std::vector<cv::Point2f> leftTag0Corners, leftTag1Corners, rightTag0Corners, rightTag1Corners;
        // printf("Determining Pos\n");
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

        // printf("Determining Dist\n");
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

                    objPts.push_back(objp);
                    leftCornersVector.push_back(leftCornerPts);
                    rightCornersVector.push_back(rightCornerPts);

                    currentPose += 2;

                    cv::imwrite(saveFoldername + "Left_" + std::to_string(currentPose / 2) + ".png", leftIm);
                    cv::imwrite(saveFoldername + "Right_" + std::to_string(currentPose / 2) + ".png", rightIm);
                    ch = 0;
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
    }

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
    exit(0);
}