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

std::vector<std::vector<cv::Point3f>> objPts;
std::vector<std::vector<cv::Point2f>> imgPts;

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
    lccv::PiCamera cam_0;
    int camId;
    bool useCan = false;

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
            }
        }
    }
    else {
        camId = 0;
        printf("No camera specified, defaulting to 0\n");
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

    createCamera(cam_0, camId);
    cam_0.startVideo();

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
    saveFoldername = "/home/pi/CalibrationImgs/" + stream.str() + "_Cam" + std::to_string(camId) + "Imgs/";

    std::filesystem::create_directories(saveFoldername.c_str());

    cv::Mat cam_0_im, vis;

    cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dict, params);

    cv::Size patternSize_(6, 8);

    cv::Size imageSize(cam_0.options->video_width, cam_0.options->video_height);

    std::vector<CalibrationPose> points = generateIntrPoints(imageSize);

    // CalibrationPose tagLoc = points[std::stoi(argv[1])];

    int currentPose = 0;

    while (currentPose < points.size()) {
        sendSerial();
        // printf("NOT PAST\n");
        if (!cam_0.getVideoFrame(cam_0_im, 99999999)) {
            std::cout << "Timeout error " << std::endl;
        }

        // printf("PAST\n");

        cv::cvtColor(cam_0_im, vis, cv::COLOR_RGB2BGR);

        CalibrationPose tagLoc = points[currentPose];

        std::string disText = "";

        if (tagLoc.direction == 1) {
            disText = "Straight";
        }
        else if (tagLoc.direction == 0) {
            disText = "Left";
        }
        else {
            disText = "Right";
        }
        cv::putText(vis, disText, cv::Point2d(0, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 5);

        cv::putText(vis, disText, cv::Point2d(0, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 0), 3);

        cv::circle(vis, tagLoc.tag0Pos, 5, cv::Scalar(127, 255, 255), 5);
        cv::circle(vis, tagLoc.tag1Pos, 5, cv::Scalar(0, 0, 255), 5);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        detector.detectMarkers(cam_0_im, markerCorners, markerIds, rejectedCandidates);

        cv::aruco::drawDetectedMarkers(vis, markerCorners, markerIds);

        std::vector<cv::Point2f> tag0Corners, tag1Corners;
        for (int i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == 0) {
                tag0Corners = markerCorners[i];
            }
            else if (markerIds[i] == 1) {
                tag1Corners = markerCorners[i];
            }
        }

        if (tag0Corners.size() > 0 && tag1Corners.size() > 0) {
            cv::Mat mean;
            cv::reduce(tag0Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag0Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));
            cv::reduce(tag1Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag1Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));

            double tag0dist = cv::norm(tagLoc.tag0Pos - tag0Pos);
            double tag1dist = cv::norm(tagLoc.tag1Pos - tag1Pos);

            if (tag0dist < 20 && tag1dist < 20) {
                cv::Mat grayIm;
                cv::cvtColor(cam_0_im, grayIm, cv::COLOR_RGB2GRAY);

                std::vector<cv::Point2f> cornerPts;
                bool found = cv::findChessboardCornersSB(grayIm, patternSize_, cornerPts);

                if (found) {
                    cv::drawChessboardCorners(vis, patternSize_, cornerPts, found);

                    objPts.push_back(objp);
                    imgPts.push_back(cornerPts);

                    currentPose++;

                    cv::imwrite(saveFoldername + "/IntrImg_" + std::to_string(currentPose) + ".png", cam_0_im);
                    ch = 0;
                    sleep(1);
                }
            }

            // printf("Tag 0 pos: %f, %f\n", tag0Pos.x, tag0Pos.y);
            // printf("Tag 1 pos: %f, %f\n", tag1Pos.x, tag1Pos.y);
        }

        // printf("%d markers detected\n", markerIds.size());

        if (useCan) {
            imageTx->transmitImage(vis);
            usleep(200000);
        }
        else {
            cv::imshow("Image", vis);
	    usleep(200000);
            ch = cv::waitKey(1) & 255;
        }

        if (ch == 'q') {
            break;
        }
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
    cam_0.stopVideo();
}
