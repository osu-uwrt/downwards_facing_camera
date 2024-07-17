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

OrientationAgent::OrientationAgent(char *id, YoloAgent *yoAgent) {
    yoloAgent = yoAgent;

    int ifIdx = if_nametoindex(id);
    if (!ifIdx) {
        throw std::system_error(errno, std::generic_category(), "if_nametoindex");
    }

    client = MicroROSClient::create(ifIdx, CANMORE_CLIENT_ID_DOWNWARDS_CAMERA, "talos");

    client.waitForAgent();

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
            if (client.getDetectionsEnabled(targetTask)) {
                yoloAgent->setTask(targetTask);
                YoloDepth inputs;
                try {
                    inputs = yoloAgent->output.pop();
                } catch (std::runtime_error &e) {
                    continue;
                }
                std::vector<Detection> detections;
                inputs.getDetections(detections);

                std::vector<CameraDetection> detectionMsg(detections.size());

                cv::Mat left, right;
                inputs.getImages(left, right);
                cv::Mat depthMap;
                inputs.getDepthMap(depthMap);

                for (Detection detection : detections) {
                    std::vector<cv::Point2i> corners;
                    cv::goodFeaturesToTrack(
                        left(cv::Rect(detection.bbox[0] - detection.bbox[2] / 2, detection.bbox[1] - detection[3] / 2)),
                        corners, 0, 0.02, -1);

                    if (corners.size() >= 5) {
                        std::vector<cv::Point3f> corners3d(corners.size());

                        std::vector<cv::Point2i> undistorted(corners.size());

                        cv::undistortPoints(corners, undistorted, lCamMat, lCamDist);
                        cv::Mat mask(80, 80, CV_8U, detection.mask);

                        for (int i = 0; i < corners.size(); i++) {
                            if (mask.at(corners[i] / 4) == 1) {
                                float depth = depthMap.at<float>(corners.at(i));

                                corners3d.push_back(cv::Point3f(undistorted[i].x, undistorted[i].y, depth));
                            }
                        }

                        if (corners3d.size() > 5) {
                            cv::Mat meanMat;
                            cv::reduce(corners3d, meanMat, 01, CV_REDUCE_AVG);

                            cv::Point3f mean(meanMat.at<float>(0), meanMat.at<float>(1), meanMat.at<float>(2));

                            cv::Mat w, u, vt;

                            cv::SVD::compute(corners3d, w, u, vt);

                            cv::Point3f normal(vt.at<float>(6), vt.t().at<float>(7), vt.t().at<float>(8));

                            // NORMAL TO QUAT

                            CameraDetection camDet;

                            camDet.score = detection.conf;
                            camDet.classId = detection.classId;
                        }
                    }
                }

                // Do the orientation things with inputs
            }
        }
    }
}