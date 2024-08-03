#include "agents/orientationAgent.hpp"

#include <iostream>
#include <net/if.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

cv::Mat redMask(cv::Mat image, cv::Mat mask) {
    cv::Mat hsv, upper, lower, newMask;
    cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(10, 255, 255), lower);
    cv::inRange(hsv, cv::Scalar(160, 0, 0), cv::Scalar(180, 255, 255), upper);
    cv::bitwise_or(upper, lower, newMask);
    cv::resize(mask, mask, cv::Size(320, 320), cv::INTER_LINEAR);
    cv::bitwise_and(newMask, mask, newMask);
    // printf("%d\n", newMask.type());
    return newMask;
}

cv::Mat blueMask(cv::Mat image, cv::Mat mask) {
    cv::Mat hsv, newMask;
    cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(90, 0, 0), cv::Scalar(120, 255, 255), newMask);
    cv::resize(mask, mask, cv::Size(320, 320), cv::INTER_LINEAR);
    cv::bitwise_and(newMask, mask, newMask);
    return newMask;
}

OrientationAgent::OrientationAgent(MicroROSClient &client, YoloAgent *yoAgent): client_(client) {
    yoloAgent = yoAgent;

    running = true;

    producing = true;

    producingThread = std::thread(&OrientationAgent::produce, this);
}

OrientationAgent::~OrientationAgent() {
    running = false;
    if (producingThread.joinable())
        producingThread.join();
}

void OrientationAgent::startProducing() {
    producing = true;
}

void OrientationAgent::stopProducing() {
    producing = false;
}

void OrientationAgent::produce() {
    int targetTask = 0;
    while (running) {
        if (producing) {
            if (client_.getDetectionsEnabled(targetTask)) {
                yoloAgent->setTask(targetTask);
                YoloDepth inputs;
                try {
                    inputs = yoloAgent->yoloOutput.pop();
                } catch (std::runtime_error &e) {
                    continue;
                }
                std::vector<Detection> lDetections, rDetections;
                inputs.getLeftDetections(lDetections);
                inputs.getRightDetections(rDetections);

                std::vector<CameraDetection> detectionMsg;

                cv::Mat left, right;
                inputs.getImages(left, right);

                for (Detection detection : lDetections) {
                    std::vector<cv::Point2i> corners;

                    corners.push_back(centroid(detection.bbox[0] * 320 - detection.bbox[2] / 160,
                                               detection.bbox[1] * 320 - detection.bbox[3] / 160));

                    std::vector<cv::Point2f> undistorted(corners.size());

                    cv::undistortPoints(corners, undistorted, lCamMat, lCamDist);
                    cv::Mat mask(80, 80, CV_8U, detection.mask);
                    cv::Mat red = redMask(left, mask);

                    CameraDetection camDet;

                    camDet.score = detection.conf;
                    camDet.classId = detection.classId;

                    camDet.pose.pose.position.x = undistorted[0].x;
                    camDet.pose.pose.position.y = undistorted[0].y;
                    camDet.pose.pose.position.z = 1;

                    detectionMsg.push_back(camDet);
                }

                for (Detection detection : rDetections) {
                    std::vector<cv::Point2i> corners;

                    corners.push_back(centroid(detection.bbox[0] * 320 - detection.bbox[2] / 160,
                                               detection.bbox[1] * 320 - detection.bbox[3] / 160));

                    std::vector<cv::Point2f> undistorted(corners.size());

                    cv::undistortPoints(corners, undistorted, rCamMat, rCamDist);
                    cv::Mat mask(80, 80, CV_8U, detection.mask);
                    cv::Mat red = redMask(right, mask);

                    CameraDetection camDet;

                    camDet.score = detection.conf;
                    camDet.classId = detection.classId;

                    camDet.pose.pose.position.x = undistorted[0].x;
                    camDet.pose.pose.position.y = undistorted[0].y;
                    camDet.pose.pose.position.z = 1;

                    detectionMsg.push_back(camDet);
                }

                client_.reportDetections(inputs.getTimeStamp(), detectionMsg);
            }
        }
    }
}
