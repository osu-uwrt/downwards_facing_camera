#ifndef DATA_UTILS
#define DATA_UTILS

#include "coral_yolo.hpp"
#include "opencv2/core.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <queue>

#include <condition_variable>

class YoloDepth {
public:
    void setImages(cv::Mat &left, cv::Mat &right) {
        leftIm = left;
        rightIm = right;
    }

    void setDetections(std::vector<Detection> &detOutput) { detections_ = detOutput; }

    void setTimestamp(timespec ts) { timestamp = ts; }

    void getImages(cv::Mat &left, cv::Mat &right) {
        left = leftIm;
        right = rightIm;
    }
    void getLeftDetections(std::vector<Detection> &detections) { detections = detections_; }

    timespec getTimeStamp() { return timestamp; }

private:
    bool depthSet, detectionsSet;
    cv::Mat leftIm, rightIm;
    std::vector<Detection> detections_;
    timespec timestamp;
};

template <typename T> class TSQueue {
private:
    // Underlying queue
    std::shared_ptr<std::queue<T>> m_queue;

    // mutex for thread synchronization
    std::mutex m_mutex;

    std::condition_variable m_cond;

public:
    TSQueue();

    TSQueue &operator=(TSQueue &rhs);

    // Pushes an element to the queue
    void push(T item);

    // Pops an element off the queue
    T pop();

    size_t size();
};

template <typename T> TSQueue<T>::TSQueue() {
    m_queue = std::make_shared<std::queue<T>>();
}

template <typename T> TSQueue<T> &TSQueue<T>::operator=(TSQueue<T> &rhs) {
    std::unique_lock<std::mutex> lhs_lock(this->m_mutex);
    std::unique_lock<std::mutex> rhs_lock(rhs.m_mutex);

    this->m_queue = rhs.m_queue;
    this->m_cond = rhs.m_cond;

    return *this;
}

// Pushes an element to the queue
template <typename T> void TSQueue<T>::push(T item) {
    // Acquire lock
    std::scoped_lock<std::mutex> lock(m_mutex);

    // Add item
    m_queue->push(item);

    m_cond.notify_one();
}

// Pops an element off the queue
template <typename T> T TSQueue<T>::pop() {
    // acquire lock
    std::unique_lock<std::mutex> lock(m_mutex);

    // retrieve item
    if (m_queue->empty()) {
        if (!m_cond.wait_for(lock, std::chrono::seconds(1), [this]() { return !m_queue->empty(); })) {
            throw std::runtime_error("Queue is empty");
        }
    }

    T item = m_queue->front();
    m_queue->pop();

    // return item
    return item;
}

template <typename T> size_t TSQueue<T>::size() {
    return m_queue->size();
}

#endif