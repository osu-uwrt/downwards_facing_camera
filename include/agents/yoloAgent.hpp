#ifndef YOLOAGENT
#define YOLOAGENT

#include <thread>
#include <condition_variable>

#include "agents/cameraAgent.hpp"
#include "tools/DataUtils.hpp"

#include "coral_yolo.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

class YoloAgent {
public:
    TSQueue<YoloDepth> yoloOutput;

    YoloAgent(CameraAgent *camAgent);
    ~YoloAgent();

    void startDetecting();
    void stopDetecting();

    void restartPID();
    void restartConnection();

    void setTask(int task);
private:
    bool running, inferencing;

    int connectionSocket, dataSocket;

    char imageBuffer[320 * 320 * 3 * 2];

    std::shared_ptr<CoralYoloItf> model;
    std::thread inferencingThread;

    CameraAgent *cameraAgent;

    pid_t yolo = 0;

    void inference();
};

#endif
