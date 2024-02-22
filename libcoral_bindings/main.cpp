#include "coral_yolo.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>

int main(int argc, char* argv[]) {
    auto yolo = createCoralYolo(
        "/home/pi/yolo_model/"
        "yolov8n-seg_full_integer_quant_edgetpu.tflite",
        80, std::stof(argv[1]));

    cv::Mat image = cv::imread("/home/pi/yolo_im.bmp", cv::IMREAD_ANYCOLOR);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    std::vector<Detection> detections = yolo->detectImage(image.data);

    std::ofstream detectionFile("Detections.csv");

    for (Detection &detection : detections) {
        detectionFile << (int)detection.bbox[0] << ',';
        detectionFile << (int)detection.bbox[1] << ',';
        detectionFile << (int)detection.bbox[2] << ',';
        detectionFile << (int)detection.bbox[3] << ',';

        detectionFile << (int)detection.classId << ',';
        detectionFile << (int)detection.conf << '\n';
    }
    detectionFile.close();

    printf("Num detections: %d\n", detections.size());

    _exit(0);
}
