#include "calibration/pointGenerator.hpp"

double tagSize = 50.0;       // in mm
double tagDistance = 460.0;  // in mm
double boardHeight = 290.0;  // in mm
double focalLen = 4.2;       // in mm
double baseline = 120.0;     // in mm
cv::Size boardSize(tagDistance + tagSize, boardHeight);

cv::Size2f sensorSize(5.08556, 3.71847);  // Calculated from 6.3mm diagonal and aspect ratio

std::vector<double> intrDepths1 = { 430 };  // Depths for 1x1 grid
std::vector<double> intrDepths2 = { 600 };  // Depths for 2x2 grid
std::vector<double> intrDepths3 = { 900 };  // Depths for 3x3 grid

std::vector<double> sterDepths2 = { 600 };        // Depths for 2x2 grid
std::vector<double> sterDepths4 = { 900, 1200 };  // Depths for 4x4 grid

double planeLocToPixelLoc(double planeLen, double planeLoc, double imageLen) {
    return planeLoc / planeLen * imageLen;
}

std::vector<CalibrationPose> generateIntrPoints(cv::Size imageSize) {
    std::vector<CalibrationPose> targetTagPoses;

    for (double depth : intrDepths1) {
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

            if (i != 1) {
                pose.tag0Pos.x += skewedDistPixelDiff / 2;
                pose.tag1Pos.x -= skewedDistPixelDiff / 2;
            }

            pose.direction = i;

            targetTagPoses.push_back(pose);
        }
    }

    for (double depth : intrDepths2) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        double yPlaneLocs[] = { 20.0 + boardSize.height / 2, planeSize.height - boardSize.height / 2 - 20.0};
        double xPlaneLocs[] = { tagSize / 2 + 20.0, planeSize.width - boardSize.width + tagSize / 2 - 20.0 };

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 3; k++) {
                    CalibrationPose pose;
                    pose.tag0Pos = cv::Point2f(planeLocToPixelLoc(planeSize.width, xPlaneLocs[j], imageSize.width),
                                               planeLocToPixelLoc(planeSize.height, yPlaneLocs[i], imageSize.height));

                    pose.tag1Pos = cv::Point2f(
                        planeLocToPixelLoc(planeSize.width, xPlaneLocs[j] + boardSize.width - tagSize, imageSize.width),
                        planeLocToPixelLoc(planeSize.height, yPlaneLocs[i], imageSize.height));

                    pose.direction = k;

                    if (k != 1) {
                        double dist = cv::norm(pose.tag0Pos - pose.tag1Pos);

                        double skewedDist = dist * cos(PI / 6.0);

                        double skewedDistPixelDiff = dist - skewedDist;

                        pose.tag0Pos.x += skewedDistPixelDiff / 2;
                        pose.tag1Pos.x -= skewedDistPixelDiff / 2;
                    }

                    targetTagPoses.push_back(pose);
                }
            }
        }
    }

    for (double depth : intrDepths3) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        double yPlaneLocs[] = {boardSize.height / 2 + 20.0, planeSize.height / 2, planeSize.height - boardSize.height / 2 - 20.0};
        double xPlaneLocs[] = { tagSize / 2 + 20.0, planeSize.width / 2 - boardSize.width / 2 + tagSize / 2,
                                planeSize.width - boardSize.width + tagSize / 2  - 20.0};

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    CalibrationPose pose;
                    pose.tag0Pos = cv::Point2f(planeLocToPixelLoc(planeSize.width, xPlaneLocs[j], imageSize.width),
                                               planeLocToPixelLoc(planeSize.height, yPlaneLocs[i], imageSize.height));

                    pose.tag1Pos = cv::Point2f(
                        planeLocToPixelLoc(planeSize.width, xPlaneLocs[j] + boardSize.width - tagSize, imageSize.width),
                        planeLocToPixelLoc(planeSize.height, yPlaneLocs[i], imageSize.height));

                    pose.direction = k;

                    if (k != 1) {
                        double dist = cv::norm(pose.tag0Pos - pose.tag1Pos);

                        double skewedDist = dist * cos(PI / 6.0);

                        double skewedDistPixelDiff = dist - skewedDist;

                        pose.tag0Pos.x += skewedDistPixelDiff / 2;
                        pose.tag1Pos.x -= skewedDistPixelDiff / 2;
                    }

                    targetTagPoses.push_back(pose);
                }
            }
        }
    }

    return targetTagPoses;
}

std::vector<CalibrationPose> generateSterPoints(cv::Size imageSize) {
    std::vector<CalibrationPose> targetTagPoses;

    double leftImShift = 120.0;

    for (double depth : sterDepths2) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        cv::Size overlapSize;
        overlapSize.width = planeSize.width - baseline;
        overlapSize.height = planeSize.height;

        double yOverlapLocs[] = { 20.0 + boardSize.height / 2, overlapSize.height - boardSize.height / 2 - 20.0};
        double xOverlapLocs[] = { tagSize / 2 + 20.0, overlapSize.width - boardSize.width + tagSize / 2  - 20.0};

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                // Even index is right cam, odd index is left cam
                for (int k = 0; k < 2; k++) {
                    CalibrationPose pose;

                    pose.direction = 0;
                    pose.tag0Pos =
                        cv::Point2f(planeLocToPixelLoc(planeSize.width, xOverlapLocs[j] + baseline * k, imageSize.width),
                                    planeLocToPixelLoc(planeSize.height, yOverlapLocs[i], imageSize.height));

                    pose.tag1Pos =
                        cv::Point2f(planeLocToPixelLoc(planeSize.width, xOverlapLocs[j] + baseline * k + boardSize.width - tagSize,
                                                       imageSize.width),
                                    planeLocToPixelLoc(planeSize.height, yOverlapLocs[i], imageSize.height));

                    targetTagPoses.push_back(pose);
                }
            }
        }
    }

    for (double depth : sterDepths4) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        cv::Size overlapSize;
        overlapSize.width = planeSize.width - baseline;
        overlapSize.height = planeSize.height;

        double betweenHeightDiff = (overlapSize.height - boardSize.height) / 3;
        double betweenWidthDiff = (overlapSize.width - boardSize.width) / 3;

        double yOverlapLocs[] = { 20.0 + boardSize.height / 2, boardSize.height / 2 + betweenHeightDiff,
                                  overlapSize.height - boardSize.height / 2 - betweenHeightDiff,
                                  overlapSize.height - boardSize.height / 2 - 20.0};
        double xOverlapLocs[] = { tagSize / 2 + 20.0, tagSize / 2 + betweenWidthDiff,
                                  overlapSize.width - boardSize.width + tagSize / 2 - betweenWidthDiff,
                                  overlapSize.width - boardSize.width + tagSize / 2  - 20.0};

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 2; k++) {
                    CalibrationPose pose;

                    pose.direction = 0;
                    pose.tag0Pos =
                        cv::Point2f(planeLocToPixelLoc(planeSize.width, xOverlapLocs[j] + baseline * k, imageSize.width),
                                    planeLocToPixelLoc(planeSize.height, yOverlapLocs[i], imageSize.height));

                    pose.tag1Pos =
                        cv::Point2f(planeLocToPixelLoc(planeSize.width, xOverlapLocs[j] + baseline * k + boardSize.width - tagSize,
                                                       imageSize.width),
                                    planeLocToPixelLoc(planeSize.height, yOverlapLocs[i], imageSize.height));

                    targetTagPoses.push_back(pose);
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

    int returnDir = 0;

    if (widthDiff > 1.15) {
        returnDir = 1;
    }
    else if (widthDiff < .85) {
        returnDir = 2;
    }

    return returnDir;
}
