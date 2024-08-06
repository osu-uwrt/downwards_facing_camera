#include <coral_yolo.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

void write_gaurentee(int fd, const void *buf, size_t buf_size) {
    const unsigned char *bufchar = (unsigned char *) buf;
    const unsigned char *bufptr = bufchar;

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

void read_gaurentee(int fd, void *buf_out, size_t buf_size) {
    unsigned char *bufchar = (unsigned char *) buf_out;
    unsigned char *bufptr = bufchar;

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

int main(int argc, char *argv[]) {
    // Create Coral
    auto model = createCoralYolo("/home/pi/robosub_2024_5_full_integer_quant_edgetpu.tflite", 5, 0.5, 0.9);

    struct sockaddr_un addr;

    int data_socket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (data_socket == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    /*
     * For portability clear the whole structure, since some
     * implementations have additional (nonstandard) fields in
     * the structure.
     */

    memset(&addr, 0, sizeof(addr));

    /* Connect socket to socket address. */

    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/buffer", sizeof(addr.sun_path) - 1);

    usleep(2500000);

    int ret = connect(data_socket, (const struct sockaddr *) &addr, sizeof(addr));
    if (ret == -1) {
        fprintf(stderr, "The server is down.\n");
        exit(EXIT_FAILURE);
    }

    const size_t readSize = 320 * 320 * 6;
    unsigned char *buffer = new unsigned char[readSize];

    while (true) {
        read_gaurentee(data_socket, buffer, readSize);

        printf("Read image\n");

        // cv::Mat lImage(320, 320, CV_8UC3, buffer);
        // cv::imwrite("test.jpg", lImage);

        usleep(20000);
        std::vector<Detection> lDetections, rDetections;

        model->preprocessImage((uint8_t *) buffer);
        model->detectImage();

        printf("Ran detection left\n");

        lDetections = model->processDetections();

        usleep(20000);

        model->preprocessImage((uint8_t *) (buffer + 320 * 320 * 3));
        model->detectImage();

        printf("Ran detection right\n");

        rDetections = model->processDetections();

        int lBufferSize = lDetections.size();
        int rBufferSize = rDetections.size();

        int totalBufferSize = lBufferSize + rBufferSize;

        printf("Total Buffer Size: %d\n", totalBufferSize);

        write_gaurentee(data_socket, &totalBufferSize, sizeof(int));

        for (Detection detection : lDetections) {
            printf("L ID %d\n", detection.classId);
            write_gaurentee(data_socket, detection.bbox, 4 * sizeof(float));
            write_gaurentee(data_socket, &detection.classId, sizeof(detection.classId));
            write_gaurentee(data_socket, &detection.conf, sizeof(detection.conf));
            write_gaurentee(data_socket, detection.mask, 80 * 80 * sizeof(bool));
        }

        for (Detection detection : rDetections) {
            printf("R ID %d\n", detection.classId);
            detection.classId += 20;
            write_gaurentee(data_socket, detection.bbox, 4 * sizeof(float));
            write_gaurentee(data_socket, &detection.classId, sizeof(detection.classId));
            write_gaurentee(data_socket, &detection.conf, sizeof(detection.conf));
            write_gaurentee(data_socket, detection.mask, 80 * 80 * sizeof(bool));
        }

        printf("Sent detections\n");
    }
}
