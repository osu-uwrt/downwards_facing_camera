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
    // printf("upper size: %d, %d\n", upper.size().width, upper.size().height);
    // printf("lower size: %d, %d\n", lower.size().width, lower.size().height);
    cv::bitwise_or(upper, lower, newMask);
    // printf("Did or\n");
    cv::resize(mask, mask, cv::Size(320, 320), cv::INTER_LINEAR);
    // printf("Did resize\n");
    cv::bitwise_and(newMask, mask, newMask);
    // printf("Did and\n");
    // printf("%d\n", newMask.type());
    return newMask;
}

OrientationAgent::OrientationAgent(MicroROSClient &client, YoloAgent *yoAgent): client_(client), yoloAgent(yoAgent) {
    cv::FileStorage cam0Intr("/home/pi/Cam0Intr.xml", cv::FileStorage::READ);
    cam0Intr["Matrix"] >> lCamMat;
    cam0Intr["DistCoeffs"] >> lCamDist;
    cam0Intr.release();

    cv::FileStorage cam1Intr("/home/pi/Cam1Intr.xml", cv::FileStorage::READ);
    cam1Intr["Matrix"] >> rCamMat;
    cam1Intr["DistCoeffs"] >> rCamDist;
    cam1Intr.release();

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
            // if (client_.getDetectionsEnabled(targetTask)) {
            // yoloAgent->setTask(targetTask);
            YoloDepth inputs;
            try {
                inputs = yoloAgent->yoloOutput.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            std::vector<Detection> detections;
            inputs.getDetections(detections);

            printf("Total detections: %d\n", detections.size());

            std::vector<CameraDetection> detectionMsg;

            cv::Mat left, right;
            inputs.getImages(left, right);

            for (Detection detection : detections) {
                CameraDetection camDet;

                cv::Rect bbox((int) (detection.bbox[0] * 320 - detection.bbox[2] * 160),
                              (int) (detection.bbox[1] * 320 - detection.bbox[3] * 160),
                              (int) (detection.bbox[2] * 320), (int) (detection.bbox[3] * 320));

                camDet.score = detection.conf;
                camDet.classId = std::to_string(detection.classId);
                std::vector<cv::Point2f> corners;
                printf("Id: %s\n", camDet.classId.c_str());
                if (detection.classId == binId || detection.classId == binId + 20) {
                    cv::Mat usedIm = left;
                    cv::Mat usedMat = lCamMat;
                    cv::Mat usedDist = lCamDist;
                    if (detection.classId > 20) {
                        usedIm = right;
                        usedMat = rCamMat;
                        usedDist = rCamDist;
                    }

                    cv::Mat mask(80, 80, CV_8U, detection.mask);
                    cv::Mat red = redMask(usedIm(bbox), mask(bbox));

                    std::vector<std::vector<cv::Point>> contours;
                    cv::findContours(red, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(bbox.x, bbox.y));

                    if (contours.size() > 0) {
                        int maxIndex = 0;
                        int maxArea = -1;
                        for (int i = 0; i < contours.size(); i++) {
                            double newArea = cv::contourArea(contours[i]);
                            if (newArea > maxArea) {
                                maxArea = newArea;
                                maxIndex = i;
                            }
                        }

                        cv::Moments moments;
                        moments = cv::moments(contours[maxIndex]);
                        corners.push_back(cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00));

                        std::vector<cv::Point2f> ray;
                        cv::undistortPoints(corners, ray, usedMat, usedDist);

                        camDet.pose.pose.position.x = ray[0].x;
                        camDet.pose.pose.position.x = ray[0].y;
                    }
                    else {
                        continue;
                    }
                }
                else {
                    continue;  // IF WE DO TABLE, THIS MIGHT WORK
                    // printf("pushing center of bbox\n");
                    corners.push_back(cv::Point2f(detection.bbox[0] * 320 - detection.bbox[2] / 160,
                                                  detection.bbox[1] * 320 - detection.bbox[3] / 160));

                    std::vector<cv::Point2f> undistorted(corners.size());
                    // printf("Undistorting points\n");

                    cv::undistortPoints(corners, undistorted, lCamMat, lCamDist);
                    // printf("undistored\n");
                    camDet.pose.pose.position.x = undistorted[0].x;
                    camDet.pose.pose.position.y = undistorted[0].y;
                }
                camDet.pose.pose.position.z = 1;

                detectionMsg.push_back(camDet);
            }

            client_.reportDetections(inputs.getTimeStamp(), detectionMsg);
        }
    }
}
