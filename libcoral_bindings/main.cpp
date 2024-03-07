#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "coral_yolo.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char* argv[]) {
    auto yolo = createCoralYolo(
        "/home/pi/"
        "Buoy_Gate_Torpedo_300_Rotation360_2575_Mirror_Nano_full_integer_quant_"
        "edgetpu.tflite",
        9, std::stof(argv[1]), std::stof(argv[2]));

    // auto yolo = createCoralYolo(
    //     "/home/pi/robert_attempt3/yolo_models/"
    //     "yolov8n_seg_rescaled_edgetpu.tflite",
    //     80, std::stof(argv[1]), std::stof(argv[2]));

    cv::Mat image =
        cv::imread("/home/pi/template_small.bmp", cv::IMREAD_ANYCOLOR);
    // cv::Mat image = cv::imread("/home/pi/yolo_im_reshape.bmp", cv::IMREAD_ANYCOLOR);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    auto start = std::chrono::high_resolution_clock::now();
    yolo->preprocessImage(image.data);
    yolo->detectImage();
    std::vector<Detection> detections = yolo->processDetections();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Took %dms\n", duration.count());

    std::ofstream detectionFile("Detections.csv");
    std::ofstream masksFile("/home/pi/masks.csv");
    // cv::FileStorage maskFile("/home/pi/masks.xml",
    //                          cv::FileStorage::Mode::WRITE);
    printf("Created\n");
    int counter = 0;

    printf("Going through detections\n");
    for (Detection& detection : detections) {
        detectionFile << detection.bbox[0] << ',';
        detectionFile << detection.bbox[1] << ',';
        detectionFile << detection.bbox[2] << ',';
        detectionFile << detection.bbox[3] << ',';

        detectionFile << detection.classId << ',';
        detectionFile << detection.conf << ',';
        for (int i = 0; i < 32; i++) {
            detectionFile << detection.maskWeights[i] << ',';
        }
        detectionFile << '\n';
        for (int i = 0; i < 6400; i++) {
            masksFile << detection.mask[i] << ',';
        }
        masksFile << '\n';
        counter++;
    }
    masksFile.close();
    detectionFile.close();

    printf("Num detections: %d\n", detections.size());

    _exit(0);
}
