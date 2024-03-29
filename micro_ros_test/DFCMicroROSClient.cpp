
#include "DFCMicroROSClient.hpp"

#include "micro_ros_transport.h"

#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <vision_msgs/msg/detection3_d_array.h>

#include <iostream>
#include <time.h>

#define AGENT_PING_TIMEOUT_MS 2000

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

    // create timer to ping the agent twice a second
    // Periodically pings the agent to make sure the connection is still alive
    RCCHECK(rclc_timer_init_default(&pingTimer_, &support_, RCL_MS_TO_NS(500), ping_timer_callback));

    // create executor
    executor_ = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));
    RCCHECK(rclc_executor_add_timer(&executor_, &pingTimer_));

    while (true) {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
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

void MicroROSClient::reportDetection(const CameraDetection &detection) {
    std::lock_guard<std::mutex> lk(detectionQueueMutex);
    detectionQueue.push_back(detection);
}

void MicroROSClient::handleQueuedDetections() {
    // Keep looping until we run out of detected objects (need to check under lock though)
    while (true) {
        CameraDetection dfcDetection;
        {
            std::lock_guard<std::mutex> lk(detectionQueueMutex);
            if (detectionQueue.empty()) {
                // No detected objects, exit
                return;
            }
            // We have something, grab it
            dfcDetection = detectionQueue.at(0);
            detectionQueue.pop_front();
        }

        // Now publish while not under lock
        vision_msgs__msg__Detection3DArray detections = {};
        detections.header.frame_id = stringToRosStr(inst->cameraFrame_);
        detections.header.stamp = timespecToRosTime(dfcDetection.ts);

        vision_msgs__msg__Detection3D detectionEntry = {};
        detectionEntry.header.frame_id = stringToRosStr(inst->cameraFrame_);
        detectionEntry.header.stamp = timespecToRosTime(dfcDetection.ts);

        vision_msgs__msg__ObjectHypothesisWithPose hypothesis = {};
        hypothesis.hypothesis.class_id = stringToRosStr(dfcDetection.classId);
        hypothesis.hypothesis.score = dfcDetection.score;
        hypothesis.pose = dfcDetection.pose;

        detectionEntry.results.data = &hypothesis;
        detectionEntry.results.size = 1;
        detectionEntry.results.capacity = 1;

        detections.detections.data = &detectionEntry;
        detections.detections.size = 1;
        detections.detections.capacity = 1;

        rcl_ret_t ret = rcl_publish(&detectedObjsPub_, &detections, NULL);
        if (ret != RCL_RET_OK) {
            std::cerr << "Failed to publish detection! rcl_publish failed with error code: " << ret << std::endl;
            // If we fail to publish, bail now so we don't run into issues where we stall in publish too long, we get
            // another detection, and we never end up breaking out of this loop
            return;
        }
    }
}

void MicroROSClient::ping_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;

    if (timer != NULL) {
        rmw_ret_t ret = rmw_uros_ping_agent(100, 5);
        if (ret != RMW_RET_OK) {
            std::cerr << "Agent Disconnected! Exiting!" << std::endl;
            exit(1);
        }
    }
}
