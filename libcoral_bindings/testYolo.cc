#include "coral/examples/file_utils.h"
#include "coral/tflite_utils.h"

#include "tensorflow/lite/interpreter.h"

int main(int argc, char *argv) {
    const std::unique_ptr<tflite::FlatBufferModel> model = coral::LoadModelOrDie("/home/pi/yolo_model/yolov8n-seg_full_integer_quant_edgetpu.tflite");

    auto edgetpu_context = coral::ContainsEdgeTpuCustomOp(*model)
                             ? coral::GetEdgeTpuContextOrDie()
                             : nullptr;

    std::unique_ptr<tflite::Interpreter> interperter = coral::MakeEdgeTpuInterpreterOrDie();

    if (interperter->AllocateTensors() != kTFLiteOk) {
        printf("Tensors failed to allocate\n");
        exit(-1);
    }
}