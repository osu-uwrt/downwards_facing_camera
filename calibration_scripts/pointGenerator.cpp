#include "calibration/pointGenerator.hpp"

double tagSize = 50.0;       // in mm
double tagDistance = 460.0;  // in mm
double boardHeight = 290.0;  // in mm
double focalLen = 4.2;       // in mm
cv::Size boardSize(tagDistance + tagSize, boardHeight);

cv::Size2f sensorSize(5.08556, 3.71847);  // Calculated from 6.3mm diagonal and aspect ratio

std::vector<double> depths1 = { 430, 600 };  // Depths for 1x1 grid
std::vector<double> depths2 = {};            // Depths for 2x2 grid
std::vector<double> depths3 = { 1200 };      // Depths for 3x3 grid

double planeLocToPixelLoc(double planeLen, double planeLoc, double imageLen) {
    return planeLoc / planeLen * imageLen;
}

std::vector<CalibrationPose> generateIntrPoints(cv::Size imageSize) {
    std::vector<CalibrationPose> targetTagPoses;

    for (double depth : depths1) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        double numPixFromWidthEdge =
            ((planeSize.width - boardSize.width) / 2 + tagSize / 2) / planeSize.width * imageSize.width;
        double numPixFromHeightEdge = imageSize.height / 2;

        CalibrationPose pose;

        pose.tag0Pos = cv::Point2f(numPixFromWidthEdge, numPixFromHeightEdge);
        pose.tag1Pos = cv::Point2f(imageSize.width - numPixFromWidthEdge, numPixFromHeightEdge);

        double dist = cv::norm(pose.tag0Pos - pose.tag1Pos);

        double skewedDist = dist * cos(PI / 6.0);

        double skewedDistPixelDiff = dist - skewedDist;

        for (int i = 0; i < 3; i++) {
            pose.tag0Pos = cv::Point2f(numPixFromWidthEdge, numPixFromHeightEdge);
            pose.tag1Pos = cv::Point2f(imageSize.width - numPixFromWidthEdge, numPixFromHeightEdge);

            if (i != 0) {
                pose.tag0Pos.x += skewedDistPixelDiff / 2;
                pose.tag1Pos.x -= skewedDistPixelDiff / 2;
            }

            pose.direction = i;

            targetTagPoses.push_back(pose);
        }
    }

    for (double depth : depths3) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        double yPlaneLocs[] = { tagSize / 2, planeSize.height / 2 - boardSize.height / 2 + tagSize / 2,
                                planeSize.height - boardSize.height + tagSize / 2 };
        double xPlaneLocs[] = { tagSize / 2, planeSize.width / 2 - boardSize.height / 2 + tagSize / 2,
                                planeSize.width - boardSize.width + tagSize / 2 };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    CalibrationPose pose;
                    pose.tag0Pos = cv::Point2f(planeLocToPixelLoc(planeSize.width, xPlaneLocs[j], imageSize.width),
                                               planeLocToPixelLoc(planeSize.height, yPlaneLocs[i], imageSize.height));
                    
                    pose.tag1Pos = cv::Point2f(planeLocToPixelLoc(planeSize.width, xPlaneLocs[j] + boardSize.width - tagSize / 2, imageSize.width),
                                               planeLocToPixelLoc(planeSize.height, yPlaneLocs[i] + boardSize.height - tagSize / 2, imageSize.height));

                    pose.direction = k;
                }
            }
        }
    }

    return targetTagPoses;
}

int direction(std::vector<cv::Point2f> tag0corners, std::vector<cv::Point2f> tag1corners) {
    double tag0width = std::abs(tag0corners[0].x - tag0corners[1].x);
    double tag1width = std::abs(tag1corners[0].x - tag1corners[1].x);

    double widthDiff = tag0width / tag1width;

    int returnDir = 1;

    if (widthDiff > 1.25) {
        returnDir = 0;
    }
    else if (widthDiff < .75) {
        returnDir = 2;
    }

    return returnDir;
}