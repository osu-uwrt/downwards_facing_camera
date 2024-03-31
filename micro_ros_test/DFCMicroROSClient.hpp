#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/pose_with_covariance.h>
#include <std_srvs/srv/set_bool.h>

#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

class ros_error : public std::runtime_error {
public:
    ros_error(int line, rcl_ret_t error):
        runtime_error("ROS Error on line " + std::to_string(line) + ": " + std::to_string(error)) {}
};

struct CameraDetection {
    CameraDetection(): score(0), classId(""), pose({}) {}

    float score;
    std::string classId;
    geometry_msgs__msg__PoseWithCovariance pose;
};

class MicroROSClient {
public:
    static MicroROSClient &create(int ifIndex, uint8_t clientId, const std::string &rosNamespace) {
        if (inst != nullptr) {
            throw new std::logic_error("Can only create one MicroROS client per process");
        }
        inst = new MicroROSClient(ifIndex, clientId, rosNamespace);
        return *inst;
    }
    // Disable copying
    MicroROSClient(MicroROSClient const &) = delete;
    MicroROSClient &operator=(MicroROSClient const &) = delete;

    void waitForAgent();
    void run();

    // ===Thread Safe Functions to be called from camera thread===
    bool getDetectionsEnabled(int &targetClassId);
    void reportDetections(const timespec &timestamp, const std::vector<CameraDetection> &detections);

private:
    MicroROSClient(int ifIndex, uint8_t clientId, const std::string &rosNamespace);
    static MicroROSClient *inst;

    static void ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
    static void detection_request_callback(const void *request_msg, void *response_msg);

    bool setRequestedMode(bool enabled, int targetClassId);
    void handleQueuedDetections();

    int clientId_;
    std::string rosNamespace_;
    std::string cameraFrame_;
    rcl_allocator_t allocator_ = {};
    rclc_support_t support_ = {};
    rclc_executor_t executor_ = {};
    rcl_node_t node_ = {};
    rcl_timer_t pingTimer_ = {};
    rcl_publisher_t detectedObjsPub_ = {};
    rcl_service_t detectionRequestSrv_ = {};

    std_srvs__srv__SetBool_Request detectionSrvReq_ = {};
    std_srvs__srv__SetBool_Response detectionSrvRsp_ = {};

    struct DetectionMsg {
        DetectionMsg(const timespec &ts, const std::vector<CameraDetection> &detections):
            ts(ts), detections(detections) {}

        timespec ts;
        std::vector<CameraDetection> detections;
    };
    std::deque<DetectionMsg> detectionQueue_;
    std::mutex detectionQueueMutex_;

    std::mutex requestedModeMutex_;
    bool detectionsEnabled_ = false;
    int targetClassId_ = 0;
    std::unique_ptr<std::promise<bool>> modeProcessedPromise_;
};
