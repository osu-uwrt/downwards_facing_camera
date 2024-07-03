
#include "agents/yoloAgent.hpp"
#include "tools/DFCMicroROSClient.hpp"
#include "tools/DataUtils.hpp"

#include "canmore/client_ids.h"

class OrientationAgent {
public:
    OrientationAgent(char *id, YoloAgent *yoAgent);

    ~OrientationAgent();

    void startProducing();

    void stopProducing();

private:
    bool producing, running;
    int ifIdx;

    MicroROSClient client;
    YoloAgent *yoloAgent;

    std::thread producingThread;

    void produce();
};