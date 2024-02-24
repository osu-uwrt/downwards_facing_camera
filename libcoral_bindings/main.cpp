#include "coral_yolo.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>

int main(int argc, char* argv[]) {
    auto yolo = createCoralYolo(
        "/home/pi/robert_attempt3/yolo_models/yolov8n_seg_rescaled_edgetpu.tflite",
        80, std::stof(argv[1]), std::stof(argv[2]));

    cv::Mat image = cv::imread("/home/pi/yolo_im_reshape.bmp", cv::IMREAD_ANYCOLOR);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Detection> detections = yolo->detectImage(image.data);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Took %dms\n", duration.count());

    std::ofstream detectionFile("Detections.csv");

    for (Detection &detection : detections) {
        detectionFile << detection.bbox[0] << ',';
        detectionFile << detection.bbox[1] << ',';
        detectionFile << detection.bbox[2] << ',';
        detectionFile << detection.bbox[3] << ',';

        detectionFile << detection.classId << ',';
        detectionFile << detection.conf << '\n';
    }
    detectionFile.close();

    printf("Num detections: %d\n", detections.size());

    _exit(0);
}
