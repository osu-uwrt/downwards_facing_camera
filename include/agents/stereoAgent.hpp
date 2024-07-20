#include <thread>


#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "agents/cameraAgent.hpp"
#include "stereoConfig.hpp"
#include "tools/DataUtils.hpp"

class StereoAgent {
public:
    TSQueue<YoloDepth> output;

    StereoAgent(char *stereoMapsFile, char *configFile, CameraAgent *camAgent);
    ~StereoAgent();

    void startStereo();
    void stopStereo();

private:
    bool running, gettingDepths;

    CameraAgent *cameraAgent;

    cv::Mat leftStereoMap1, leftStereoMap2, rightStereoMap1, rightStereoMap2, Q;

    cv::Ptr<cv::StereoSGBM> leftStereo, rightStereo;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter;

    std::thread depthThread;

    config stereoConfig;

    config configFromFile(char *configFile);

    void getDepths();
};