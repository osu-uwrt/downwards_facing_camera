#include "coral_yolo.hpp"

#include <iostream>

#include "coral/examples/file_utils.h"
#include "coral/tflite_utils.h"
#include "tensorflow/lite/interpreter.h"
#include "third_party/eigen3/Eigen/Core"

static float iou(float* bbox1, float* bbox2) {
    float area1 = bbox1[2] * bbox1[3] * 224 * 224;
    float area2 = bbox2[2] * bbox2[3] * 224 * 224;

    /* If points are (x1, y1) (a1, b1) and (x2, y2) (a2, b2)
       Then the intersection width would be the furthest left "right edge"
       minus the furthest right "left edge"
    */
    float xx =
        std::min<float>(bbox1[0] - (bbox1[2] / 2), bbox2[0] - (bbox2[2] / 2));
    float yy =
        std::max<float>(bbox1[1] - (bbox1[3] / 2), bbox2[1] - (bbox2[3] / 2));
    float aa =
        std::min<float>(bbox1[0] + (bbox1[2] / 2), bbox2[0] + (bbox1[2] / 2));
    float bb =
        std::min<float>(bbox1[1] + (bbox1[3] / 2), bbox2[1] + (bbox1[3] / 2));

    float intersectionWidth = std::max<float>(0, (aa - xx) * 224);
    float intersectionHeight = std::max<float>(0, (bb - yy) * 224);

    float intersectionArea = intersectionWidth * intersectionHeight;

    float unionArea = area1 + area2 - intersectionArea;
    float iou = intersectionArea / unionArea;

    return iou;
}

class CoralYolo : public CoralYoloItf {
   public:
    std::string model_path_;
    int num_classes_;
    int8_t int_min_conf_;
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

        const TfLiteQuantizationParams& bboxOutputParams =
            interpreter->output_tensor(0)->params;
        bboxOutputScale = bboxOutputParams.scale;
        bboxZeroPoint = bboxOutputParams.zero_point;

        int_min_conf_ = (int8_t)(min_conf_ / bboxOutputScale + bboxZeroPoint);

        const TfLiteQuantizationParams& segOutputParams =
            interpreter->output_tensor(1)->params;
        segOutputScale = segOutputParams.scale;
        segZeroPoint = segOutputParams.zero_point;
    }

    void preprocessImage(uint8_t* image) {
        for (int i = 0; i < 307200; i++) {
            input[i] = (int8_t)(((float)image[i] / (float)255) / inputScale +
                                inputZeroPoint);
        }
    }
    void detectImage() override {
        if (interpreter->Invoke() != kTfLiteOk) {
            printf("Image could not be invoked\n");
            exit(-1);
        }
        // printf("Copying output0\n");
        memcpy(output0,
               coral::MutableTensorData<int8_t>(*interpreter->output_tensor(0))
                   .data(),
               interpreter->output_tensor(0)->bytes);
        for (int i = 0; i < 204800; i++) {
            int8_t result =
                coral::MutableTensorData<int8_t>(*interpreter->output_tensor(0))
                    .data()[i];

            output1[i] = postProcessValue(result, false);
        }
        // printf("Copying to eigen matrix\n");
        masks = Eigen::Map<Eigen::Matrix<float, 6400, 32, Eigen::RowMajor>>(output1);
        // masks = masks.transpose();
        // printf("Transposed\n");
    }

    std::vector<Detection> processDetections() {
        std::vector<Detection> detections;
        for (int i = 0; i < 2100; i++) {
            Detection detection;
            int8_t maxConf = -127;
            for (int j = 0; j < num_classes_; j++) {
                int8_t result = output0[i + (j + 4) * 2100];
                // Output tensor shape is (116, 8400)
                if (result >= int_min_conf_ && result > maxConf) {
                    detection.classId = j;
                    maxConf = result;
                }
            }
            if (maxConf >= min_conf_) {
                // THIS WAY SHOULD BE RIGHT (AND IT WAS!!!!)
                detection.conf = postProcessValue(maxConf, true);
                detection.bbox[0] = postProcessValue(output0[i], true);
                detection.bbox[1] = postProcessValue(output0[i + 2100], true);
                detection.bbox[2] =
                    postProcessValue(output0[i + 2 * 2100], true);
                detection.bbox[3] =
                    postProcessValue(output0[i + 3 * 2100], true);
                Eigen::VectorXf maskWeights(32);
                for (int j = 0; j < 32; j++) {
                    maskWeights[j] = postProcessValue(
                        output0[i + num_classes_ + j * 2100], true);
                }
                nmsWithMask(detections, detection, iou_thresh_, maskWeights);
            }
        }
        return detections;
    }

   private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    std::shared_ptr<edgetpu::EdgeTpuContext> edgetpu_context;
    int8_t *input, *output0;
    float inputZeroPoint, inputScale, bboxZeroPoint, bboxOutputScale,
        segZeroPoint, segOutputScale;
    float output1[204800];
    Eigen::MatrixXf masks;

    float postProcessValue(int8_t value, bool isOutput0) {
        if (isOutput0) {
            return ((float)value - bboxZeroPoint) * bboxOutputScale;
        } else {
            return ((float)value - segZeroPoint) * segOutputScale;
        }
    }


    void processMask(Eigen::MatrixXf& weights, Detection& detection) {
        Eigen::Matrix<float, 6400, 1> outputMask;
        outputMask.noalias() = masks * weights;
        outputMask = outputMask.unaryExpr(std::ref(sigmoid));
        outputMask = outputMask.unaryExpr(std::ref(fixValue));
        // Eigen::ArrayXf array;
        // outputMask = outputMask.unaryExpr([](float elem) {
        //     return elem > (float) 0.5 ? (float) 1.0 : (float) 0.0;
        // });
        // outputMask = outputMask.unaryExpr(&fixValue);
        // outputMask = (outputMask.array() > 0.5).select(1.0, outputMask);
        Eigen::Map<Eigen::MatrixXf>(detection.mask, 6400, 1) = outputMask;
        // std::copy(outputMask.data(), outputMask.data() + 6400, detection.mask);
    }

    void nmsWithMask(std::vector<Detection>& detections,
                            Detection& newDetection, float iouThreshold,
                            Eigen::VectorXf& maskWeights) {
        bool replaced = false;
        for (int i = 0; i < detections.size(); i++) {
            if (detections.at(i).classId == newDetection.classId) {
                float calculatedIOU =
                    iou(detections.at(i).bbox, newDetection.bbox);
                if (calculatedIOU > iouThreshold) {
                    if (detections.at(i).conf < newDetection.conf) {
                        detections.erase(detections.begin() + i);
                        // printf("Calculating mask\n");
                        processMask(maskWeights, newDetection);
                        detections.emplace_back(newDetection);
                    }
                    replaced = true;
                    break;
                }
            }
        }
        if (!replaced) {
            processMask(maskWeights, newDetection);
            detections.emplace_back(newDetection);
        }
    }
};

std::shared_ptr<CoralYoloItf> createCoralYolo(std::string model_path,
                                              int num_classes, float min_conf,
                                              float iou_thresh) {
    return std::make_shared<CoralYolo>(model_path, num_classes, min_conf,
                                       iou_thresh);
}
