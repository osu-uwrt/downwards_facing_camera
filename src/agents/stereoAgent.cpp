#include <agents/stereoAgent.hpp>
#include <opencv2/imgproc.hpp>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "ryml_all.hpp"

StereoAgent::StereoAgent(char *stereoMapsFile, char *configFile, CameraAgent *camAgent) {
    cv::FileStorage mapsFile("/home/pi/StereoMaps.xml", cv::FileStorage::READ);
    mapsFile["Left_Stereo_Map_x"] >> leftStereoMap1;
    mapsFile["Left_Stereo_Map_y"] >> leftStereoMap2;
    mapsFile["Right_Stereo_Map_x"] >> rightStereoMap1;
    mapsFile["Right_Stereo_Map_y"] >> rightStereoMap2;
    mapsFile["Q"] >> Q;
    mapsFile.release();

    stereoConfig = configFromFile(configFile);

    leftStereo =
        cv::StereoSGBM::create(stereoConfig.minDisparity, stereoConfig.numDisparities, stereoConfig.blockSize,
                               stereoConfig.P1, stereoConfig.P2, stereoConfig.disp12MaxDiff, stereoConfig.preFilterCap,
                               stereoConfig.uniquenessRatio, stereoConfig.speckleWindowSize, stereoConfig.speckleRange);

    rightStereo = cv::ximgproc::createRightMatcher(leftStereo);
    wslFilter = cv::ximgproc::createDisparityWLSFilter(rightStereo);
    wslFilter->setLambda(stereoConfig.lambda);
    wslFilter->setSigmaColor(stereoConfig.sigma);

    running = true;

    cameraAgent = camAgent;

    depthThread = std::thread(&StereoAgent::getDepths, this);
}

StereoAgent::~StereoAgent() {
    running = false;
    if (depthThread.joinable())
        depthThread.join();
}

void StereoAgent::startStereo() {
    gettingDepths = true;
}

void StereoAgent::stopStereo() {
    gettingDepths = false;
}

config StereoAgent::configFromFile(char *configFile) {
    config configuration;

    ryml::Parser parser;
    ryml::Tree yamlTree = parser.parse_in_arena(configFile);
    configuration.P1 = std::stoi(yamlTree["P1"].val());
    configuration.P2 = std::stoi(yamlTree["P2"].val());
    configuration.blockSize = std::stoi(yamlTree["blockSize"].val());
    configuration.baseline = std::stoi(yamlTree["baseline"].val());
    configuration.disp12MaxDiff = std::stoi(yamlTree["disp12MaxDiff"].val());
    configuration.lambda = std::stoi(yamlTree["lambda"].val());
    configuration.minDisparity = std::stoi(yamlTree["minDisparity"].val());
    configuration.numDisparities = std::stoi(yamlTree["numDisparities"].val());
    configuration.preFilterCap = std::stoi(yamlTree["preFilterCap"].val());
    configuration.preFilterSize = std::stoi(yamlTree["preFilterSize"].val());
    configuration.sigma = std::stoi(yamlTree["sigma"].val());
    configuration.speckleRange = std::stoi(yamlTree["speckleRange"].val());
    configuration.speckleWindowSize = std::stoi(yamlTree["speckleWindowSize"].val());
    configuration.textureThreshold = std::stoi(yamlTree["textureThreshold"].val());
    configuration.uniquenessRatio = std::stoi(yamlTree["uniquenessRatio"].val());
    configuration.focalLen = std::stof(yamlTree["focalLen"].val());

    return configuration;
}

void StereoAgent::getDepths() {
    while (running) {
        if (gettingDepths && output.size() < 1) {
            YoloDepth imageHandle;
            try {
                imageHandle = cameraAgent->imageQueue.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            cv::Mat images[2];
            imageHandle.getImages(images[0], images[1]);

            cv::Mat leftNice, rightNice, leftDisp, rightDisp, disparity;
            cv::remap(images[0], leftNice, leftStereoMap1, leftStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
            cv::remap(images[1], rightNice, rightStereoMap1, rightStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT,
                      0);

            leftStereo->compute(leftNice, rightNice, leftDisp);
            rightStereo->compute(rightNice, leftNice, rightDisp);
            leftDisp /= 16.0;
            rightDisp /= 16.0;
            wslFilter->filter(leftDisp, leftNice, disparity, rightDisp);

            cv::Mat depthMap =
                disparity * stereoConfig.baseline * stereoConfig.focalLen / (disparity + stereoConfig.doffs + 1E-6);

            imageHandle.setDepth(depthMap);

            output.push(imageHandle);
        }
    }
}
