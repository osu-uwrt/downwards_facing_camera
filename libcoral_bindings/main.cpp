#include "coral_yolo.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>

int detectionType = 0; // 0: No classes 1: Table 2: Bins

int main(int argc, char* argv[]) {
    auto yolo = createCoralYolo(
        "/home/pi/BinZoomCrop_full_integer_quant_edgetpu.tflite",
        80, std::stof(argv[1]), std::stof(argv[2]));

    cv::Mat image = cv::imread("/home/pi/test.jpg", cv::IMREAD_ANYCOLOR);
    cv::resize(image, image, cv::Size(320, 320), cv::INTER_LINEAR);
    //cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

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

    printf("Num detections: %d\n", detections.size());

    _exit(0);
}
