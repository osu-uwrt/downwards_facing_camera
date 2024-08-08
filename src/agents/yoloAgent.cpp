#include <agents/yoloAgent.hpp>
#include <signal.h>

bool write_gaurentee(int fd, const void *buf, size_t buf_size) {
    const unsigned char *bufchar = (unsigned char *) buf;
    const unsigned char *bufptr = bufchar;

    while (bufptr < bufchar + buf_size) {
        size_t remaining = buf_size - (bufptr - bufchar);
        if (remaining > 2048) {
            remaining = 2048;
        }
        int ret = write(fd, bufptr, remaining);
        if (ret == 0) {
            fprintf(stderr, "Socket Closed\n");
            return false;
        }
        else if (ret < 0) {
            // perror("write");
            return false;
        }
        bufptr += ret;
    }
    return true;
}

bool read_gaurentee(int fd, void *buf_out, size_t buf_size) {
    unsigned char *bufchar = (unsigned char *) buf_out;
    unsigned char *bufptr = bufchar;

    while (bufptr < bufchar + buf_size) {
        size_t remaining = buf_size - (bufptr - bufchar);
        int ret = read(fd, bufptr, remaining);
        if (read == 0) {
            fprintf(stderr, "Socket Closed\n");
            return false;
        }
        else if (ret < 0) {
            perror("read");
            printf("Timed out\n");
            return false;
        }
        bufptr += ret;
    }

    return true;
}

YoloAgent::YoloAgent(CameraAgent *camAgent): running(true), inferencing(false), cameraAgent(camAgent) {
    system("rm /tmp/buffer");
    connectionSocket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (connectionSocket == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    restartPID();  // Start child process
    restartConnection();

    inferencingThread = std::thread(&YoloAgent::inference, this);
}

YoloAgent::~YoloAgent() {
    running = false;
    if (inferencingThread.joinable())
        inferencingThread.join();
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
            cv::copyMakeBorder(images[0], images[0], 70, 70, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            cv::resize(images[1], images[1], cv::Size(320, 180));
            cv::copyMakeBorder(images[1], images[1], 70, 70, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            printf("Sending image 1\n");
            bool fail = write_gaurentee(dataSocket, images[0].data, size_t(320 * 320 * 3));
            if (fail) {
                restartPID();
                restartConnection();
                continue;
            }
            printf("Sending image 2\n");
            fail = write_gaurentee(dataSocket, images[1].data, size_t(320 * 320 * 3));
            if (fail) {
                restartPID();
                restartConnection();
                continue;
            }
            int detSize = 0;

            printf("Waiting for response\n");
            read_gaurentee(dataSocket, &detSize, sizeof(int));

            printf("Got response\n");
            std::vector<Detection> detections;

            for (int i = 0; i < detSize; i++) {
                Detection detection;
                fail = read_gaurentee(dataSocket, detection.bbox, 4 * sizeof(float));
                if (fail) {
                    restartPID();
                    restartConnection();
                    continue;
                }
                fail = read_gaurentee(dataSocket, &detection.classId, sizeof(detection.classId));
                if (fail) {
                    restartPID();
                    restartConnection();
                    continue;
                }
                fail = read_gaurentee(dataSocket, &detection.conf, sizeof(detection.conf));
                if (fail) {
                    restartPID();
                    restartConnection();
                    continue;
                }
                fail = read_gaurentee(dataSocket, detection.mask, 80 * 80 * sizeof(bool));
                if (fail) {
                    restartPID();
                    restartConnection();
                    continue;
                }

                detections.push_back(detection);
            }
            imageHandle.setDetections(detections);

            imageHandle.setImages(images[0], images[1]);

            yoloOutput.push(imageHandle);
        }
    }
}

void YoloAgent::setTask(int task) {
    model->setTask(task);
}

void YoloAgent::restartPID() {
    if (yolo != 0) {
        kill(yolo, SIGKILL);
    }
    yolo = fork();
    if (yolo == 0) {
        execvp("/home/pi/osu-uwrt/downwards_facing_camera/build/yoloInterpreter");
    }
    else if (yolo == -1) {
        restartPID();
    }
}

void YoloAgent::restartConnection() {
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

    printf("Accepted\n");

    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    ret = setsockopt(dataSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(timeval));
}