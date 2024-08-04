#include <agents/yoloAgent.hpp>

void write_gaurentee(int fd, const void* buf, size_t buf_size) {
    const unsigned char* bufchar = (unsigned char*) buf;
    const unsigned char* bufptr = bufchar;

    while (bufptr < bufchar + buf_size) {
        size_t remaining = buf_size - (bufptr - bufchar);
        int ret = write(fd, bufptr, remaining);
        if (ret == 0) {
            fprintf(stderr, "Socket Closed\n");
            exit(EXIT_FAILURE);
        }
        else if (ret < 0) {
            perror("write");
            exit(EXIT_FAILURE);
        }
        bufptr += ret;
    }
}

void read_gaurentee(int fd, void* buf_out, size_t buf_size) {
    unsigned char* bufchar = (unsigned char*) buf_out;
    unsigned char* bufptr = bufchar;

    while (bufptr < bufchar + buf_size) {
        size_t remaining = buf_size - (bufptr - bufchar);
        int ret = read(fd, bufptr, remaining);
        if (read == 0) {
            fprintf(stderr, "Socket Closed\n");
            exit(EXIT_FAILURE);
        }
        else if (ret < 0) {
            perror("read");
            exit(EXIT_FAILURE);
        }
        bufptr += ret;
    }
}

YoloAgent::YoloAgent(CameraAgent *camAgent): running(true), inferencing(false), cameraAgent(camAgent) {
    connectionSocket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (connectionSocket == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    /*
     * For portability clear the whole structure, since some
     * implementations have additional (nonstandard) fields in
     * the structure.
     */

    struct sockaddr_un name;
    memset(&name, 0, sizeof(name));

    /* Bind socket to socket name. */

    name.sun_family = AF_UNIX;
    strncpy(name.sun_path, "/tmp/buffer", sizeof(name.sun_path) - 1);

    int ret = bind(connectionSocket, (const struct sockaddr *) &name, sizeof(name));
    if (ret == -1) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    /*
     * Prepare for accepting connections. The backlog size is set
     * to 20. So while one request is being processed other requests
     * can be waiting.
     */

    ret = listen(connectionSocket, 20);
    if (ret == -1) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    dataSocket = accept(connectionSocket, NULL, NULL);
    if (dataSocket == -1) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    struct timeval tv;
    tv.tv_sec = 2;

    setsockopt(dataSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    inferencingThread = std::thread(&YoloAgent::inference, this);
}

YoloAgent::~YoloAgent() {
    running = false;
    if (inferencingThread.joinable())
        inferencingThread.join();
    if (watchdog.joinable())
        watchdog.join();
    _Exit(0);
}

void YoloAgent::startDetecting() {
    inferencing = true;
}

void YoloAgent::stopDetecting() {
    inferencing = false;
}

void YoloAgent::inference() {
    while (running) {
        // if (inferencing && )
        if (inferencing) {
            YoloDepth imageHandle;
            try {
                imageHandle = cameraAgent->imageQueue.pop();
            } catch (std::runtime_error &e) {
                continue;
            }
            cv::Mat images[2];
            imageHandle.getImages(images[0], images[1]);

            cv::resize(images[0], images[0], cv::Size(320, 180));
            cv::copyMakeBorder(images[0], images[0], 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            cv::resize(images[1], images[1], cv::Size(320, 180));
            cv::copyMakeBorder(images[1], images[1], 70, 70, 0, 0, cv::BORDER_DEFAULT, cv::Scalar(0, 0, 0));

            write_gaurentee(dataSocket, images[0].data, 320 * 320 * 3);
            write_gaurentee(dataSocket, images[1].data, 320 * 320 * 3);
            int detSize = 0;

            read_gaurentee(dataSocket, &detSize, sizeof(int));

            std::vector<Detection> detections(detSize);

            for (int i = 0; i < detSize; i++) {
                Detection detection;
                read_gaurentee(dataSocket, detection.bbox, sizeof(detection.bbox));
                read_gaurentee(dataSocket, &detection.classId, sizeof(detection.classId));
                read_gaurentee(dataSocket, &detection.conf, sizeof(detection.conf));
                read_gaurentee(dataSocket, detection.mask, sizeof(detection.mask));

                detections.push_back(detection);
            }

            imageHandle.setDetections(detections);

            imageHandle.setImages(images[0], images[1]);

            yoloOutput.push(imageHandle);

            m_cond.notify_all();
        }
    }
}

void YoloAgent::setTask(int task) {
    model->setTask(task);
}
