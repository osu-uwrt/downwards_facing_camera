#include <fstream>
#include <iostream>

#include "coral/examples/file_utils.h"
#include "coral/tflite_utils.h"
#include "tensorflow/lite/interpreter.h"

struct Detection {
    int classId;
    int conf;
    int bbox[4];
};

// std::unique_ptr<tflite::FlatBufferModel> model;
// std::unique_ptr<tflite::Interpreter> interpreter;
// std::shared_ptr<edgetpu::EdgeTpuContext> edgetpu_context;

int main(int argc, char *argv[]) {
    auto model = std::move(
        coral::LoadModelOrDie("/home/pi/yolo_model/"
                              "yolov8n-seg_full_integer_quant_edgetpu.tflite"));
    if (model == nullptr) {
        printf("Model not created\n");
    }
    auto edgetpu_context = coral::ContainsEdgeTpuCustomOp(*model)
                               ? coral::GetEdgeTpuContextOrDie()
                               : nullptr;

    if (edgetpu_context == nullptr) {
        printf("Context not found\n");
    }
    printf("Making interpreter\n");
    // coral::MakeEdgeTpuInterpreterOrDie(*model, edgetpu_context.get());
    printf("Didn't die");
    auto interpreter =
        coral::MakeEdgeTpuInterpreterOrDie(*model, edgetpu_context.get());
    printf("Here1");
    auto status = interpreter->AllocateTensors();
    printf("Here2");
    if (status != kTfLiteOk) {
        printf("Tensors failed to allocate %d\n", status);
        exit(-1);
    }
    printf("Here");
    auto input =
        coral::MutableTensorData<uint8_t>(*interpreter->input_tensor(0));

    // void detectImage(unsigned char *image) {
    //     memcpy(input.data(), image, input.size());
    //     if (interpreter->Invoke() != kTfLiteOk) {
    //         printf("Image could not be invoked\n");
    //         exit(-1);
    //     }
    // }

    // std::vector<Detection> processBoxes(absl::Span<const float> results) {
    //     std::vector<Detection> detections;
    //     for (int i = 0; i < 8400; i++) {
    //         Detection detection;
    //         detection.bbox[0] = results[116 * i];
    //         detection.bbox[1] = results[116 * i + 1];
    //         detection.bbox[2] = results[116 * i + 2];
    //         detection.bbox[3] = results[116 * i + 3];
    //         float maxConf = -1;
    //         for (int j = 0; j < num_classes_; j++) {
    //             if (results[116 * i + j + 4] > min_conf_ &&
    //                 results[116 * i + j + 4] >
    //                     maxConf) {  // Output tensor shape is (116, 8400)
    //                 detection.classId = j;
    //             }
    //         }
    //         if (maxConf != -1) {
    //             detections.emplace_back(detection);
    //         }
    //     }
    //     return detections;
    // }

    // yolo.LoadModel();

    // std::vector<uint8_t> image_data(1228800);
    // std::ifstream file("/home/pi/yolo_model/yolo_im.rgb", std::ios::binary);
    // file.read(reinterpret_cast<char *>(image_data.data()), 1228800);

    // yolo.detectImage(image_data.data());

    // const auto *outputTensor = interpreter->output_tensor(0);
}