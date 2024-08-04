#include "coral_yolo.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <unistd.h>

int main(int argc, char *argv[]) {
    auto model = createCoralYolo("/home/pi/robosub_2024_1_full_integer_quant_edgetpu.tflite", 4, 0.8, 0.8);
    cv::Mat image = cv::imread("/home/pi/Ex_Stereo_Imgs/Ltorp1.png");
    image.convertTo(image, CV_8UC3);

    cv::resize(image, image, cv::Size(320, 320));

    while (true) {
        model->preprocessImage(image.data);
        model->detectImage();

        model->processDetections();

        printf("Testing\n");

        usleep(100000);
    }
}