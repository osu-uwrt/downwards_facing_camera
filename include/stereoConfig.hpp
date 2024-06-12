#include <opencv2/calib3d.hpp>

struct config {
    int baseline, P1, P2, blockSize, disp12MaxDiff, doffs, lambda, minDisparity, numDisparities, preFilterCap,
        preFilterSize, preFilterType, sigma, speckleRange, speckleWindowSize, textureThreshold, uniquenessRatio;
    float focalLen;
    bool SGBM;
};

// Returns config struct from file in specified location
config configFromFile();