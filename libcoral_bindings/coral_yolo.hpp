#pragma once

#include <memory>
#include <vector>

struct Detection {
    int classId;
    float conf;
    float bbox[4];
};

class CoralYoloItf {
   public:
    virtual std::vector<Detection> detectImage(uint8_t* image) = 0;
};

static std::shared_ptr<CoralYoloItf> createCoralYolo(std::string model_path,
                                                     int num_classes,
                                                     float min_conf);
