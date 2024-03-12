#pragma once

#include <memory>
#include <vector>

struct Detection {
    int classId;
    float conf;
    float bbox[4];
    bool mask[6400];
};

class CoralYoloItf {
   public:
    virtual void detectImage() = 0;

    virtual void preprocessImage(uint8_t* image) = 0;

    virtual std::vector<Detection> processDetections() = 0;
};

std::shared_ptr<CoralYoloItf> createCoralYolo(std::string model_path,
                                              int num_classes, float min_conf,
                                              float iou_thresh);
