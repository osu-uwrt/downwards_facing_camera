#include <agents/stereoAgent.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>

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

    std::ifstream file(configFile);

    std::string content(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());

    ryml::Tree yamlTree;
    ryml::parse_in_place(content, &yamlTree);
    
    configuration.P1 = std::stoi(std::string(yamlTree["P1"].val().data()));
    configuration.P2 = std::stoi(std::string(yamlTree["P2"].val().data()));
    configuration.blockSize = std::stoi(std::string(yamlTree["blockSize"].val().data()));
    configuration.baseline = std::stoi(std::string(yamlTree["baseline"].val().data()));
    configuration.disp12MaxDiff = std::stoi(std::string(yamlTree["disp12MaxDiff"].val().data()));
    configuration.lambda = std::stoi(std::string(yamlTree["lambda"].val().data()));
    configuration.minDisparity = std::stoi(std::string(yamlTree["minDisparity"].val().data()));
    configuration.numDisparities = std::stoi(std::string(yamlTree["numDisparities"].val().data()));
    configuration.preFilterCap = std::stoi(std::string(yamlTree["preFilterCap"].val().data()));
    configuration.preFilterSize = std::stoi(std::string(yamlTree["preFilterSize"].val().data()));
    configuration.sigma = std::stoi(std::string(yamlTree["sigma"].val().data()));
    configuration.speckleRange = std::stoi(std::string(yamlTree["speckleRange"].val().data()));
    configuration.speckleWindowSize = std::stoi(std::string(yamlTree["speckleWindowSize"].val().data()));
    configuration.textureThreshold = std::stoi(std::string(yamlTree["textureThreshold"].val().data()));
    configuration.uniquenessRatio = std::stoi(std::string(yamlTree["uniquenessRatio"].val().data()));
    configuration.focalLen = std::stof(std::string(yamlTree["focalLen"].val().data()));

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
