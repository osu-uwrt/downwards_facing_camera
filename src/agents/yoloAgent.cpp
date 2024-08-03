#include <agents/yoloAgent.hpp>

YoloAgent::YoloAgent(char *tfliteFile, int numClasses, double conf, double iou, CameraAgent *camAgent) {
    model = createCoralYolo(tfliteFile, numClasses, conf, iou);

    cameraAgent = camAgent;

    running = true;

    inferencingThread = std::thread(&YoloAgent::inference, this);
}

YoloAgent::~YoloAgent() {
    running = false;
    if (inferencingThread.joinable())
        inferencingThread.join();

    _Exit(0);
}

void YoloAgent::startDetecting() {
    inferencing = true;
}

void YoloAgent::stopDetecting() {
    inferencing = false;
}

void YoloAgent::inference() {
    while (running) {
        // if (inferencing && )
        if (inferencing) {
            YoloDepth imageHandle;
            try {
                imageHandle = cameraAgent->output.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            std::vector<Detection> lDetections, rDetections;

            cv::Mat images[2];
            imageHandle.getImages(images[0], images[1]);

            cv::Mat resized;
            cv::resize(images[0], resized, cv::Size(320, 180));
            cv::copyMakeBorder(resized, resized, 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            model->preprocessImage(resized.data);
            model->detectImage();

            lDetections = model->processDetections();

            cv::resize(images[1], resized, cv::Size(320, 180));
            cv::copyMakeBorder(resized, resized, 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            model->preprocessImage(resized.data);
            model->detectImage();

            rDetections = model->processDetections();

            imageHandle.setLeftDetections(lDetections);
            imageHandle.setRightDetections(rDetections);

            yoloOutput.push(imageHandle);
        }
    }
}

void YoloAgent::setTask(int task) {
    model->setTask(task);
}