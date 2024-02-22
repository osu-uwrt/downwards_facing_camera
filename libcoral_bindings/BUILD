# This build file is designed to be part of the libcoral repository
# The script file in this directory will create a docker container to compile this
# Additionally, you can manually build this BAZEL project by adding this to the google-coral/libcoral repository,
# inside of the coral directory.

package(
    default_visibility = ["//visibility:public"],
)

cc_binary(
    name = "coral_yolo",
    srcs = ["coral_yolo.cpp", "coral_yolo.hpp"],
    linkopts = ["-shared"],
    deps = [
        "//coral/examples:file_utils",
        "//coral:tflite_utils",
        "//coral/classification:adapter",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@glog",
        "@libedgetpu//tflite/public:oss_edgetpu_direct_all",  # buildcleaner: keep
        "@org_tensorflow//tensorflow/lite:framework",
        "@org_tensorflow//tensorflow/lite/c:common",
    ],
)
