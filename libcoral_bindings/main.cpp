#include "coral_yolo.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>

int detectionType = 0; // 0: No classes 1: Table 2: Bins

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

int main(int argc, char* argv[]) {
    auto yolo = createCoralYolo(
        "/home/pi/BinZoomCrop_full_integer_quant_edgetpu.tflite",
        11, std::stof(argv[1]), std::stof(argv[2]));

    cv::Mat image = cv::imread("/home/pi/test.jpg", cv::IMREAD_ANYCOLOR);
    cv::copyMakeBorder(image, image, 140, 140, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::resize(image, image, cv::Size(320, 320), cv::INTER_LINEAR);
    // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    auto start = std::chrono::high_resolution_clock::now();
    yolo->preprocessImage(image.data);
    yolo->detectImage();
    std::vector<Detection> detections = yolo->processDetections();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Took %dms\n", duration.count());

    std::ofstream detectionFile("Detections.csv");
    std::ofstream masksFile("/home/pi/masks.csv");

    for (Detection &detection : detections) {
        detectionFile << detection.bbox[0] << ',';
        detectionFile << detection.bbox[1] << ',';
        detectionFile << detection.bbox[2] << ',';
        detectionFile << detection.bbox[3] << ',';

        detectionFile << detection.classId << ',';
        detectionFile << detection.conf << '\n';
        for (int i = 0; i < 6400; i++) {
            masksFile << detection.mask[i] << ',';
        }
        masksFile << '\n';
    }
    detectionFile.close();
    masksFile.close();

    cv::Mat mask(80, 80, CV_8U, detections[0].mask);
    cv::Rect bbox((detections[0].bbox[0] - detections[0].bbox[2] / 2) * 320, (detections[0].bbox[1] - detections[0].bbox[3] / 2) * 320, 
                  (detections[0].bbox[2]) * 320, (detections[0].bbox[3]) * 320);

    // cv::Mat hsvIm, colorMask;
    // cv::cvtColor(image, hsvIm, cv::COLOR_RGB2HSV);
    // cv::inRange(hsvIm, cv::Scalar(170, 0, 0), cv::Scalar(270, 255, 255), colorMask);
    // printf("%d\n", colorMask.type());
    // cv::resize(mask, mask, cv::Size(320, 320), cv::INTER_LINEAR);
    // cv::bitwise_and(colorMask, mask, mask);
    cv::Mat red, blue;
    red = redMask(image, mask);
    red *= 255;
    cv::cvtColor(red, red, cv::COLOR_GRAY2BGR);
    cv::rectangle(red, bbox, cv::Scalar(0, 255, 0));
    
    blue = blueMask(image, mask);
    blue *= 255;
    cv::cvtColor(blue, blue, cv::COLOR_GRAY2BGR);
    cv::rectangle(blue, bbox, cv::Scalar(0, 255, 0));

    cv::imshow("Image", red);
    cv::waitKey(0);
    cv::imshow("Image", blue);
    cv::waitKey(0);

    printf("Num detections: %d\n", detections.size());

    _exit(0);
}
