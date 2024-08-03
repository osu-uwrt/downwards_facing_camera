#ifndef YOLOAGENT
#define YOLOAGENT

#include <thread>
#include <condition_variable>

#include "agents/cameraAgent.hpp"
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

    std::condition_variable m_cond;

    std::shared_ptr<CoralYoloItf> model;
    std::thread inferencingThread;
    std::thread watchdog;

    CameraAgent *cameraAgent;

    void inference();

    void ensureInference();
};

#endif
