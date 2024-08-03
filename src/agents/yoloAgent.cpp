#include <agents/yoloAgent.hpp>

YoloAgent::YoloAgent(char *tfliteFile, int numClasses, double conf, double iou, CameraAgent *camAgent):
    running(true), inferencing(false), cameraAgent(camAgent) {
    model = createCoralYolo(tfliteFile, numClasses, conf, iou);

    inferencingThread = std::thread(&YoloAgent::inference, this);
    watchdog = std::thread(&YoloAgent::ensureInference, this);
}

YoloAgent::~YoloAgent() {
    running = false;
    if (inferencingThread.joinable())
        inferencingThread.join();
    if (watchdog.joinable()) 
        watchdog.join(); 
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
                imageHandle = cameraAgent->imageQueue.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            std::vector<Detection> lDetections, rDetections;

            cv::Mat images[2];
            imageHandle.getImages(images[0], images[1]);

            cv::resize(images[0], images[0], cv::Size(320, 180));
            cv::copyMakeBorder(images[0], images[0], 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            model->preprocessImage(images[0].data);
            model->detectImage();

            lDetections = model->processDetections();

            cv::resize(images[1], images[1], cv::Size(320, 180));
            cv::copyMakeBorder(images[1], images[1], 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            model->preprocessImage(images[1].data);
            model->detectImage();

            rDetections = model->processDetections();

            imageHandle.setLeftDetections(lDetections);
            imageHandle.setRightDetections(rDetections);

            imageHandle.setImages(images[0], images[1]);

            yoloOutput.push(imageHandle);

            m_cond.notify_all();
        }
    }
}

YoloAgent::ensureInference() {
    std::mutex mutex;
    std::unique_lock<std::mutex> lock(mutex);
    while (running) {
        if (inferencing) {
            if (!m_cond.wait_for(lock, std::chrono::seconds(5), [this]() { return false; })) {
                exit(0);
            }
        }
    }
}

void YoloAgent::setTask(int task) {
    model->setTask(task);
}