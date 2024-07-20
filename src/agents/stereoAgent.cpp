#include <agents/stereoAgent.hpp>
#include <opencv2/imgproc.hpp>

StereoAgent::StereoAgent(char *stereoMapsFile, char *configFile, CameraAgent *camAgent) {
    cv::FileStorage mapsFile("/home/pi/StereoMaps.xml", cv::FileStorage::READ);
    mapsFile["Left_Stereo_Map_x"] >> leftStereoMap1;
    mapsFile["Left_Stereo_Map_y"] >> leftStereoMap2;
    mapsFile["Right_Stereo_Map_x"] >> rightStereoMap1;
    mapsFile["Right_Stereo_Map_y"] >> rightStereoMap2;
    mapsFile["Q"] >> Q;
    mapsFile.release();

    stereoConfig = configFromFile(configFile);

    // TODO ENSURE THIS WORKS
    if (stereoConfig.SGBM) {
        leftStereo = cv::StereoSGBM::create(
            stereoConfig.minDisparity, stereoConfig.numDisparities, stereoConfig.blockSize, stereoConfig.P1,
            stereoConfig.P2, stereoConfig.disp12MaxDiff, stereoConfig.preFilterCap, stereoConfig.uniquenessRatio,
            stereoConfig.speckleWindowSize, stereoConfig.speckleRange);
    }
    else {
        leftStereo = cv::StereoBM::create(stereoConfig.numDisparities, stereoConfig.blockSize);
        leftStereo->setPreFilterType(stereoConfig.preFilterType);
        leftStereo->setPreFilterSize(stereoConfig.preFilterSize);
        leftStereo->setPreFilterCap(stereoConfig.preFilterCap);
        leftStereo->setTextureThreshold(stereoConfig.textureThreshold);
    }

    leftStereo->setMinDisparity(stereoConfig.minDisparity);
    leftStereo->setNumDisparities(stereoConfig.numDisparities);
    leftStereo->setBlockSize(stereoConfig.blockSize);
    leftStereo->setUniquenessRatio(stereoConfig.uniquenessRatio);
    leftStereo->setSpeckleRange(stereoConfig.speckleRange);
    leftStereo->setSpeckleWindowSize(stereoConfig.speckleWindowSize);
    leftStereo->setDisp12MaxDiff(stereoConfig.disp12MaxDiff);

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