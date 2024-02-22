#include "coral_yolo.hpp"

#include <iostream>

#include "coral/examples/file_utils.h"
#include "coral/tflite_utils.h"
#include "tensorflow/lite/interpreter.h"

static float iou(float* bbox1, float* bbox2) {
    float area1 = bbox1[2] * bbox1[3];
    float area2 = bbox2[2] * bbox2[3];

    /* If points are (x1, y1) (a1, b1) and (x2, y2) (a2, b2)
       Then the intersection width would be the furthest left "right edge"
       minus the furthest right "left edge"
    */
    float intersectionWidth = std::max<float>(
        0, std::min(bbox1[0] + bbox1[2] / 2, bbox2[0] + bbox2[2] / 2) -
               std::max(bbox1[0] - bbox1[2] / 2, bbox2[0] - bbox2[2] / 2));
    float intersectionHeight = std::max<float>(
        0, std::min(bbox1[1] + bbox1[3] / 2, bbox2[1] + bbox2[3] / 2) -
               std::max(bbox1[1] - bbox1[3] / 2, bbox2[1] - bbox2[3] / 2));

    float intersectionArea = intersectionWidth * intersectionHeight;

    float unionArea = area1 + area2 - intersectionArea;
    float iou = intersectionArea / unionArea;

    return iou;
}

static void nms(std::vector<Detection>& detections, Detection& newDetection,
                float iouThreshold) {
    bool replaced = false;
    for (int i = 0; i < detections.size(); i++) {
        if (detections.at(i).classId == newDetection.classId) {
            float calculatedIOU = iou(detections.at(i).bbox, newDetection.bbox);
            printf("Calculated IOU: %f\n", calculatedIOU);
            if (calculatedIOU <= iouThreshold) {
                if (detections.at(i).conf < newDetection.conf) {
                    detections.erase(detections.begin() + i);
                    detections.insert(detections.begin() + i, newDetection);
                }
                replaced = true;
                break;
            }
        }
    }
    if (!replaced) {
        detections.emplace_back(newDetection);
    }
}

class CoralYolo : public CoralYoloItf {
   public:
    std::string model_path_;
    int num_classes_;
    float min_conf_, iou_thresh_;

    CoralYolo(std::string model_path, int num_classes, float min_conf,
              float iou_thresh) {
        model_path_ = model_path;
        num_classes_ = num_classes;
        min_conf_ = min_conf;
        iou_thresh_ = iou_thresh;

        LoadModel();
    }

    ~CoralYolo() { printf("I'm being destroyed :)\n"); }

    void LoadModel() {
        model = coral::LoadModelOrDie(model_path_);
        if (model == nullptr) {
            printf("Model not created\n");
        }
        edgetpu_context = coral::ContainsEdgeTpuCustomOp(*model)
                              ? coral::GetEdgeTpuContextOrDie()
                              : nullptr;

        if (edgetpu_context == nullptr) {
            printf("Context not found\n");
        }
        interpreter =
            coral::MakeEdgeTpuInterpreterOrDie(*model, edgetpu_context.get());
        auto status = interpreter->AllocateTensors();
        if (status != kTfLiteOk) {
            printf("Tensors failed to allocate %d\n", status);
            exit(-1);
        }
        input = coral::MutableTensorData<int8_t>(*interpreter->input_tensor(0))
                    .data();

        const TfLiteQuantizationParams& inputParams =
            interpreter->input_tensor(0)->params;
        inputScale = inputParams.scale;
        inputZeroPoint = inputParams.zero_point;

        const TfLiteQuantizationParams& outputParams =
            interpreter->output_tensor(0)->params;
        outputScale = outputParams.scale;
        outputZeroPoint = outputParams.zero_point;
    }

    void preprocessImage(uint8_t* image) {
        for (int i = 0; i < 1228800; i++) {
            input[i] = (int8_t)(((float)image[i] / (float)255) / inputScale +
                                inputZeroPoint);
        }
    }

    float postProcessValue(int8_t value) {
        return ((float)value - outputZeroPoint) * outputScale;
    }

    std::vector<Detection> detectImage(uint8_t* image) override {
        preprocessImage(image);
        if (interpreter->Invoke() != kTfLiteOk) {
            printf("Image could not be invoked\n");
            exit(-1);
        }
        printf("Successful copy\n");

        return processBoxes(
            coral::TensorData<int8_t>(*interpreter->output_tensor(0)));
    }

    std::vector<Detection> processBoxes(absl::Span<const int8_t> results) {
        std::vector<Detection> detections;
        printf("Size: %d\n", results.size());
        int count = 0;
        for (int i = 0; i < 8400; i++) {
            Detection detection;
            detection.bbox[0] = postProcessValue(results.at(i));
            detection.bbox[1] = postProcessValue(results.at(i + 8400));
            detection.bbox[2] = postProcessValue(results.at(i + 2 * 8400));
            detection.bbox[3] = postProcessValue(results.at(
                i + 3 * 8400));  // THIS WAY SHOULD BE RIGHT (AND IT WAS!!!!)

            float maxConf = -1;
            for (int j = 0; j < num_classes_; j++) {
                float result = postProcessValue(results.at(i + (j + 4) * 8400));
                if (count == 0 && result > 1) {
                    printf("This shouldn't happen lol %f\n", result);
                    count++;
                }
                if (result <= 1.0 && result >= min_conf_ &&
                    result > maxConf) {  // Output tensor shape is (116, 8400)
                    detection.classId = j;
                    maxConf = result;
                }
            }
            if (maxConf >= min_conf_) {
                detection.conf = maxConf;
                nms(detections, detection, iou_thresh_);
            }
        }
        return detections;
    }

   private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    std::shared_ptr<edgetpu::EdgeTpuContext> edgetpu_context;
    int8_t* input;
    float inputZeroPoint, inputScale, outputZeroPoint, outputScale;
};

std::shared_ptr<CoralYoloItf> createCoralYolo(std::string model_path,
                                              int num_classes, float min_conf,
                                              float iou_thresh) {
    return std::make_shared<CoralYolo>(model_path, num_classes, min_conf,
                                       iou_thresh);
}
