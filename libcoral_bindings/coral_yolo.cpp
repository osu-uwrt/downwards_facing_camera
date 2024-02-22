#include <iostream>

#include "coral/examples/file_utils.h"
#include "coral/tflite_utils.h"
#include "tensorflow/lite/interpreter.h"

#include "coral_yolo.hpp"

class CoralYolo: public CoralYoloItf {
   public:
    std::string model_path_;
    int num_classes_;
    float min_conf_;

    CoralYolo(std::string model_path, int num_classes, float min_conf) {
        model_path_ = model_path;
        num_classes_ = num_classes;
        min_conf_ = min_conf;

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

        printf("Output dims: %d, %d, %d, %d\n",
               interpreter->output_tensor(0)->dims[0],
               interpreter->output_tensor(0)->dims[1],
               interpreter->output_tensor(0)->dims[2],
               interpreter->output_tensor(0)->dims[3]);

        printf("Input type: %d\n", interpreter->input_tensor(0)->type);
        printf("Input size: %d\n", interpreter->input_tensor(0)->bytes);

        printf("Scale: %f\nZero Point: %f\n", outputScale, outputZeroPoint);

        // printf("Num inputs: %d", interpreter->inputs().size());
    }

    void preprocessImage(uint8_t* image) {
        for (int i = 0; i < 1228800; i++) {
            input[i] = (int8_t) (((float)image[i] / (float) 255) / inputScale + inputZeroPoint);
            // input[i] = (int8_t) 0;
        }
    }

    float postProcessValue(int8_t value) {
        return ((float)value - outputZeroPoint) * outputScale;
    }

    std::vector<Detection> detectImage(uint8_t* image) override {
        preprocessImage(image);
        // printf("Pixel 1 value %d %d %d\n",
        // coral::MutableTensorData<int8_t>(*interpreter->input_tensor(0)).data()[248520],
        // coral::MutableTensorData<int8_t>(*interpreter->input_tensor(0)).data()[248521],
        //        coral::MutableTensorData<int8_t>(*interpreter->input_tensor(0)).data()[248522]);
        //        // At x: 280, y: 130 (Should be 205, 210, 213)
        // coral::ReadFileToOrDie("/home/pi/yolo_model/yolo_im.rgb",
        // reinterpret_cast<char*>(input), 1228800);
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
        // std::ofstream outputFile("OutputFile.csv");
        printf("Size: %d\n", results.size());
        int count = 0;
        for (int i = 0; i < 8400; i++) {
            Detection detection;
            detection.bbox[0] = postProcessValue(results.at(i));
            detection.bbox[1] = postProcessValue(results.at(i + 8400));
            detection.bbox[2] = postProcessValue(results.at(i + 2 * 8400));
            detection.bbox[3] = postProcessValue(
                results.at(i + 3 * 8400));  // THIS WAY SHOULD BE RIGHT
            // detection.bbox[0] = postProcessValue(results.at(116 * i));
            // detection.bbox[1] = postProcessValue(results.at(116 * i + 1));
            // detection.bbox[2] = postProcessValue(results.at(116 * i + 2));
            // detection.bbox[3] = postProcessValue(results.at(116 * i + 3));
            // outputFile << detection.bbox[0] << ',';
            // outputFile << detection.bbox[1] << ',';
            // outputFile << detection.bbox[2] << ',';
            // outputFile << detection.bbox[3] << ',';

            // if (i == 0) {
            //     printf("%d %d %d %d\n", detection.bbox[0], detection.bbox[1],
            //            detection.bbox[2], detection.bbox[3]);
            // }

            float maxConf = -1;
            for (int j = 0; j < num_classes_; j++) {
                float result = postProcessValue(results.at(i + (j + 4) * 8400));
                // float result = postProcessValue(results.at(116 * i + j + 4));
                // outputFile << result << ',';
                if (count == 0 && result > 1) {
                    printf("This shouldn't happen lol %f\n", result);
                    count++;
                }
                if (result <= 1.0 && result >= min_conf_ &&
                    result > maxConf) {  // Output tensor shape is (116, 8400)
                    detection.classId = j;
                    // printf("Got in here\n");
                    maxConf = result;
                }
            }
            // outputFile << detection.classId;
            // outputFile << '\n';
            if (maxConf >= min_conf_) {
                detection.conf = maxConf;
                detections.emplace_back(detection);
            }
        }
        // outputFile.close();
        return detections;
    }

   private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;
    std::shared_ptr<edgetpu::EdgeTpuContext> edgetpu_context;
    int8_t* input;
    float inputZeroPoint, inputScale, outputZeroPoint, outputScale;
};

std::shared_ptr<CoralYoloItf> createCoralYolo(std::string model_path, int num_classes, float min_conf) {
    return std::make_shared<CoralYolo>(model_path, num_classes, min_conf);
}
