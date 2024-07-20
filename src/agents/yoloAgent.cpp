#include <agents/yoloAgent.hpp>

YoloAgent::YoloAgent(char *tfliteFile, int numClasses, double conf, double iou, StereoAgent *stAgent) {
    model = createCoralYolo(tfliteFile, numClasses, conf, iou);

    stereoAgent = stAgent;

    running = true;

    inferencingThread = std::thread(&YoloAgent::inference, this);
}

YoloAgent::~YoloAgent() {
    running = false;
    if (inferencingThread.joinable())
        inferencingThread.join();

    _exit(0);
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
                imageHandle = stereoAgent->output.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            cv::Mat images[2];
            imageHandle.getImages(images[0], images[1]);

            cv::Mat resized;
            cv::resize(images[0], resized, cv::Size(320, 180));
            cv::copyMakeBorder(resized, resized, 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            model->preprocessImage(resized.data);
            model->detectImage();

            std::vector<Detection> detections;
            detections = model->processDetections();
            imageHandle.setDetections(detections);

            yoloOutput.push(imageHandle);
        }
    }
}

void YoloAgent::setTask(int task) {
    model->setTask(task);
}