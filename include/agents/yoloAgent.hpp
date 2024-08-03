#ifndef YOLOAGENT
#define YOLOAGENT

#include <thread>

#include "agents/stereoAgent.hpp"
#include "tools/DataUtils.hpp"

#include "coral_yolo.hpp"

class YoloAgent {
public:
    TSQueue<YoloDepth> yoloOutput;

    YoloAgent(char *tfliteFile, int numClasses, double conf, double iou, CameraAgent *camAgent);
    ~YoloAgent();

    void startDetecting();
    void stopDetecting();

    void setTask(int task);
private:
    bool running, inferencing;

    std::shared_ptr<CoralYoloItf> model;
    std::thread inferencingThread;

    CameraAgent *cameraAgent;

    void inference();
};

#endif
