
#include "DFCMicroROSClient.hpp"

#include "micro_ros_transport.h"

#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>
#include <vision_msgs/msg/detection3_d_array.h>

#include <iostream>
#include <time.h>

#define AGENT_PING_TIMEOUT_MS 2000
#define AGENT_KEEPALIVE_TIMEOUT_MS 100
#define AGENT_KEEPING_PING_ATTEMPTS 5

#define RCCHECK(fn)                                                                                                    \
    {                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                   \
            throw ros_error(__LINE__, temp_rc);                                                                        \
    }

MicroROSClient *MicroROSClient::inst = nullptr;

MicroROSClient::MicroROSClient(int ifIndex, uint8_t clientId, const std::string &rosNamespace):
    clientId_(clientId), rosNamespace_(rosNamespace) {
    micro_ros_transport_init(ifIndex, clientId);
}

void MicroROSClient::waitForAgent() {
    rcl_ret_t ret;

    do {
        ret = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1);
    } while (ret != RCL_RET_OK);
}

void MicroROSClient::run() {
    allocator_ = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

    // create node
    RCCHECK(rclc_node_init_default(&node_, "downwards_facing_camera", rosNamespace_.c_str(), &support_));

    // create detections publisher and populate the frame id
    cameraFrame_ = rosNamespace_ + "/downwards_facing_camera/left_optical";
    RCCHECK(rclc_publisher_init_default(&detectedObjsPub_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(vision_msgs, msg, Detection3DArray),
                                        "detected_objects"));

    RCCHECK(rclc_service_init_default(&detectionRequestSrv_, &node_,
                                      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "downwards_camera/mode"));

    // create timer to ping the agent twice a second
    // Periodically pings the agent to make sure the connection is still alive
    RCCHECK(rclc_timer_init_default(&pingTimer_, &support_, RCL_MS_TO_NS(500), ping_timer_callback));

    // create executor
    executor_ = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 2, &allocator_));
    RCCHECK(rclc_executor_add_timer(&executor_, &pingTimer_));
    RCCHECK(rclc_executor_add_service(&executor_, &detectionRequestSrv_, &detectionSrvReq_, &detectionSrvRsp_,
                                      detection_request_callback));

    while (true) {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
        handleQueuedDetections();
    }
}

static inline rosidl_runtime_c__String stringToRosStr(const std::string &str) {
    rosidl_runtime_c__String rosStr = { .data = (char *) str.c_str(), .size = str.size(), .capacity = str.capacity() };
    return rosStr;
}

static inline builtin_interfaces__msg__Time timespecToRosTime(const timespec &ts) {
    builtin_interfaces__msg__Time time = { .sec = (int32_t) ts.tv_sec, .nanosec = (uint32_t) ts.tv_nsec };
    return time;
}

bool MicroROSClient::getDetectionsEnabled(int &targetClassId) {
    std::lock_guard<std::mutex> lk(requestedModeMutex_);
    if (modeProcessedPromise_ != nullptr) {
        modeProcessedPromise_->set_value(true);
        modeProcessedPromise_.reset();
    }
    if (detectionsEnabled_) {
        targetClassId = targetClassId_;
    }
    return detectionsEnabled_;
}

bool MicroROSClient::setRequestedMode(bool enabled, int targetClassId) {
    std::future<bool> fut;
    {
        std::lock_guard<std::mutex> lk(requestedModeMutex_);
        detectionsEnabled_ = enabled;
        targetClassId_ = targetClassId;
        modeProcessedPromise_ = std::make_unique<std::promise<bool>>();
        fut = modeProcessedPromise_->get_future();
    }

    // Wait until the future is fulfilled (the camera process thread fetches the enabled value again)
    return fut.get();
}

void MicroROSClient::reportDetections(const timespec &timestamp, const std::vector<CameraDetection> &detections) {
    std::lock_guard<std::mutex> lk(detectionQueueMutex_);
    detectionQueue_.emplace_back(timestamp, detections);
}

void MicroROSClient::handleQueuedDetections() {
    // Keep looping until we either fail or run out of objects (that check must be under lock though)
    rcl_ret_t ret = RCL_RET_OK;
    while (ret == RCL_RET_OK) {
        // Needs to be pointer since we can't declare a null reference
        DetectionMsg *nextMsg = nullptr;
        {
            std::lock_guard<std::mutex> lk(detectionQueueMutex_);
            if (detectionQueue_.empty()) {
                // No detected objects, exit
                return;
            }
            // We have something, grab it
            nextMsg = &detectionQueue_.at(0);
            // It's safe to not remove it yet since this is the only thing reading from the queue
            // We will remove it once we're done (to avoid excess copying)
        }

        // Publish while not under lock
        vision_msgs__msg__Detection3DArray detections = {};
        detections.header.frame_id = stringToRosStr(inst->cameraFrame_);
        detections.header.stamp = timespecToRosTime(nextMsg->ts);

        // Copy all the reported detections into an array
        std::vector<vision_msgs__msg__Detection3D> detectionArr(nextMsg->detections.size());
        std::vector<vision_msgs__msg__ObjectHypothesisWithPose> hypothesisArr(nextMsg->detections.size());
        for (int i = 0; i < detectionArr.size(); i++) {
            auto &detInfo = nextMsg->detections.at(i);
            auto &hypothesis = hypothesisArr.at(i);
            auto &detOut = detectionArr.at(i);

            detOut.header.frame_id = stringToRosStr(inst->cameraFrame_);
            detOut.header.stamp = timespecToRosTime(nextMsg->ts);

            hypothesis.hypothesis.class_id = stringToRosStr(detInfo.classId);
            hypothesis.hypothesis.score = detInfo.score;
            hypothesis.pose = detInfo.pose;

            detOut.results.data = &hypothesis;
            detOut.results.size = 1;
            detOut.results.capacity = 1;
        }

        // Assign array to message
        detections.detections.data = detectionArr.data();
        detections.detections.size = detectionArr.size();
        detections.detections.capacity = detectionArr.size();

        ret = rcl_publish(&detectedObjsPub_, &detections, NULL);

        {
            // Finally remove the item we processed back under lock
            // Again, this is safe since we're the only one popping from the queue, and it avoid lots of copying
            std::lock_guard<std::mutex> lk(detectionQueueMutex_);
            detectionQueue_.pop_front();
        }
    }

    // If we fail to publish, bail now so we don't run into issues where we stall in publish too long, we
    // get another detection, and we never end up breaking out of this loop
    std::cerr << "Failed to publish detection! rcl_publish failed with error code: " << ret << std::endl;
}

void MicroROSClient::ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;

    if (timer != NULL) {
        rmw_ret_t ret = rmw_uros_ping_agent(AGENT_KEEPALIVE_TIMEOUT_MS, AGENT_KEEPING_PING_ATTEMPTS);
        if (ret != RMW_RET_OK) {
            std::cerr << "Agent Disconnected! Exiting!" << std::endl;
            exit(1);
        }
    }
}

void MicroROSClient::detection_request_callback(const void *request_msg, void *response_msg) {
    std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *) request_msg;
    std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *) response_msg;

    // TODO: Actually pass the real class ID
    bool enable = req_in->data;
    int targetClassId = 0;

    if (enable) {
        std::cout << "Enabling Detections of Class " << targetClassId << std::endl;
    }
    else {
        std::cout << "Disabling Detections" << std::endl;
    }

    bool successful = inst->setRequestedMode(enable, targetClassId);

    // Always Successful
    res_in->success = successful;
    res_in->message.data = NULL;
    res_in->message.size = 0;
    res_in->message.capacity = 0;
}
