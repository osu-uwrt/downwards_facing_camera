#ifndef POINT_GENERATOR_H
#define POINT_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <math.h>

#include "tools/mySerial.h"

#define PI 3.14159265

struct CalibrationPose {
    cv::Point2f tag0Pos;
    cv::Point2f tag1Pos;
    int direction; // 0: Straight, 1: Left, 2: Right
};

inline mySerial serialPort("/dev/ttyAMA4", 600);
std::vector<CalibrationPose> generateIntrPoints(cv::Size imageSize);

std::vector<CalibrationPose> generateSterPoints(cv::Size imageSize);

int direction(std::vector<cv::Point2f> tag0corners, std::vector<cv::Point2f> tag1corners);

#endif