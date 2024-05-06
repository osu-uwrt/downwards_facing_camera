#include "calibration/pointGenerator.hpp"

double tagSize = 50.0; // in mm
double tagDistance = 460.0; // in mm
double boardHeight = 290.0; // in mm
double focalLen = 4.2; // in mm
cv::Size boardSize(tagDistance + tagSize, boardHeight); 

cv::Size2f sensorSize(5.08556, 3.71847); // Calculated from 6.3mm diagonal and aspect ratio

std::vector<double> depths1 = {430}; // Depths for 1x1 grid
std::vector<double> depths2 = {}; // Depths for 2x2 grid
std::vector<double> depths3 = {1200}; // Depths for 3x3 grid

std::vector<std::pair<cv::Point2f, cv::Point2f>> generateIntrPoints(cv::Size imageSize) {
    std::vector<std::pair<cv::Point2f, cv::Point2f>> targetTagPoses; 

    for (double depth : depths1) {
        cv::Size planeSize;
        planeSize.width = depth * sensorSize.width / focalLen;
        planeSize.height = depth * sensorSize.height / focalLen;

        double numPixFromWidthEdge = (planeSize.width - boardSize.width + tagSize / 2) / planeSize.width * imageSize.width;
        double numPixFromHeightEdge = (planeSize.height - boardSize.height) / planeSize.height * imageSize.height;

        cv::Point2f tag0Pos(numPixFromWidthEdge, numPixFromHeightEdge);
        cv::Point2f tag1Pos(imageSize.width - numPixFromWidthEdge, numPixFromHeightEdge);

        targetTagPoses.push_back({tag0Pos, tag1Pos});
    }

    return targetTagPoses;

}