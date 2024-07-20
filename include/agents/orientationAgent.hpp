
#include "agents/yoloAgent.hpp"
#include "tools/DFCMicroROSClient.hpp"
#include "tools/DataUtils.hpp"

#include "canmore/client_ids.h"

class OrientationAgent {
public:
    OrientationAgent(MicroROSClient &client, YoloAgent *yoAgent);

    ~OrientationAgent();

    void startProducing();

    void stopProducing();

private:
    bool producing, running;
    int ifIdx;

    // TODO: SET THESE
    cv::Mat lCamMat, lCamDist;

    MicroROSClient &client_;
    YoloAgent *yoloAgent;

    std::thread producingThread;

    void produce();
};
