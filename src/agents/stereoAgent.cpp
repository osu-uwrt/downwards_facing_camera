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

    std::string content((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

    ryml::Tree yamlTree;
    ryml::parse_in_place(c4::substr(content), &yamlTree);
    
    c4::csubstr P1 = yamlTree["P1"].val();
    configuration.P1 = std::stoi(std::string(P1.data()).c_str(0, P1.size()));
    c4::csubstr P2 = yamlTree["P2"].val();
    configuration.P2 = std::stoi(std::string(P2.data()).c_str(0, P2.size()));
    c4::csubstr baseline = yamlTree["baseline"].val();
    configuration.baseline = std::stoi(std::string(baseline.data()).c_str(0, baseline.size()));
    c4::csubstr blockSize = yamlTree["blockSize"].val();
    configuration.blockSize = std::stoi(std::string(blockSize.data()).c_str(0, blockSize.size()));
    c4::csubstr disp12MaxDiff = yamlTree["disp12MaxDiff"].val();
    configuration.disp12MaxDiff = std::stoi(std::string(disp12MaxDiff.data()).c_str(0, disp12MaxDiff.size()));
    c4::csubstr lambda = yamlTree["lambda"].val();
    configuration.lambda = std::stoi(std::string(lambda.data()).c_str(0, lambda.size()));
    c4::csubstr minDisparity = yamlTree["minDisparity"].val();
    configuration.minDisparity = std::stoi(std::string(minDisparity.data()).c_str(0, minDisparity.size()));
    c4::csubstr numDisparities = yamlTree["numDisparities"].val();
    configuration.numDisparities = std::stoi(std::string(numDisparities.data()).c_str(0, numDisparities.size()));
    c4::csubstr preFilterCap = yamlTree["preFilterCap"].val();
    configuration.preFilterCap = std::stoi(std::string(preFilterCap.data()).c_str(0, preFilterCap.size()));
    c4::csubstr preFilterSize = yamlTree["preFilterSize"].val();
    configuration.preFilterSize = std::stoi(std::string(preFilterSize.data()).c_str(0, preFilterSize.size()));
    c4::csubstr sigma = yamlTree["sigma"].val();
    configuration.sigma = std::stoi(std::string(sigma.data()).c_str(0, preFilterSize()));
    c4::csubstr speckleRange = yamlTree["speckleRange"].val();
    configuration.speckleRange = std::stoi(std::string(speckleRange.data()).c_str(speckleRange.size()));
    c4::csubstr speckleWindowSize = yamlTree["speckleWindowSize"].val();
    configuration.speckleWindowSize = std::stoi(std::string(speckleWindowSize.data()).c_str(0, speckleWindowSize.size()));
    c4::csubstr textureThreshold = yamlTree["textureThreshold"].val();
    configuration.textureThreshold = std::stoi(std::string(textureThreshold.data()).c_str(0, textureThreshold.size()));
    c4::csubstr uniquenessRatio = yamlTree["uniquenessRatio"].val();
    configuration.uniquenessRatio = std::stoi(std::string(uniquenessRatio.data()).c_str(0, uniquenessRatio.size()));
    c4::csubstr focalLen = yamlTree["focalLen"].val();
    configuration.focalLen = std::stof(std::string(focalLen.data()).c_str(0, focalLen.size()));

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
