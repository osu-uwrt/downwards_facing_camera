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
                std::vector<Detection> detections;
                inputs.getDetections(detections);

                std::vector<CameraDetection> detectionMsg;

                cv::Mat left, right;
                inputs.getImages(left, right);
                cv::Mat depthMap;
                inputs.getDepthMap(depthMap);

                for (Detection detection : detections) {
                    std::vector<cv::Point2i> corners;
                    cv::goodFeaturesToTrack(left(cv::Rect(detection.bbox[0] * 320 - detection.bbox[2] / 2 * 320,
                                                          detection.bbox[1] * 320 - detection[3] / 2 * 320)),
                                            corners, 0, 0.02, 40);

                    if (corners.size() >= 5) {
                        std::vector<cv::Point3f> corners3d(corners.size());

                        std::vector<cv::Point2f> undistorted(corners.size());

                        cv::undistortPoints(corners, undistorted, lCamMat, lCamDist);
                        cv::Mat mask(80, 80, CV_8U, detection.mask);
                        cv::Mat red = redMask(left, mask);

                        for (int i = 0; i < corners.size(); i++) {
                            corners.at(i) += cv::Point2i(detection.bbox[0] * 320, detection.bbox[1] * 320);
                        }

                        for (int i = 0; i < corners.size(); i++) {
                            if (mask.at(corners[i] * 4) == 1) {
                                float depth = depthMap.at<float>(corners.at(i).y, corners.at(i).x);

                                corners3d.push_back(cv::Point3f(undistorted[i].x, undistorted[i].y, 1.) * depth);
                            }
                        }

                        if (corners3d.size() > 5) {
                            cv::Mat meanMat;
                            cv::reduce(corners3d, meanMat, 01, cv::REDUCE_AVG);

                            cv::Point3f mean(meanMat.at<float>(0), meanMat.at<float>(1), meanMat.at<float>(2));

                            cv::Mat w, u, vt;

                            cv::SVD::compute(corners3d, w, u, vt);

                            cv::Point3f normal(vt.at<float>(6), vt.at<float>(7), vt.at<float>(8));

                            // NORMAL TO QUAT

                            if (normal.z > 0) {
                                normal = -normal;
                            }

                            normal /= cv::norm(normal);

                            auto axis = normal.cross(cv::Point3f(0, 0, 1));
                            double axisLen = cv::norm(axis);

                            double quat[4];
                            double angle;

                            if (axisLen == 0) {
                                axis = cv::Point3f(1, 0, 0);
                                angle = M_PI;
                            }
                            else {
                                axis /= axisLen;
                                angle = std::acos(normal.dot(cv::Point3f(0, 0, 1)));
                            }

                            quat[0] = axis.x * std::sin(angle / 2);
                            quat[1] = axis.y * std::sin(angle / 2);
                            quat[2] = axis.z * std::sin(angle / 2);
                            quat[3] = std::cos(angle / 2);

                            CameraDetection camDet;

                            camDet.score = detection.conf;
                            camDet.classId = detection.classId;

                            camDet.pose.pose.position.x = mean.x;
                            camDet.pose.pose.position.y = mean.y;
                            camDet.pose.pose.position.z = mean.z;
                            camDet.pose.pose.orientation.w = quat[3];
                            camDet.pose.pose.orientation.x = quat[0];
                            camDet.pose.pose.orientation.y = quat[1];
                            camDet.pose.pose.orientation.z = quat[2];

                            detectionMsg.push_back(camDet);
                        }
                    }
                }

                client_.reportDetections(inputs.getTimeStamp(), detectionMsg);
            }
        }
    }
}
