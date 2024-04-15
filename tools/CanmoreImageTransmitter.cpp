#include "tools/CanmoreImageTransmitter.hpp"

#include "canmore/camera_feed_interface.h"
#include "canmore/crc32.h"
#include "canmore/msg_encoding.h"
#include "canmore/protocol.h"

#include <jerror.h>
#include <jpeglib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>

struct canmore_destination_mgr {
    struct jpeg_destination_mgr pub;   // Public destination info
    CanmoreImageTransmitter *imageTx;  // Reference to this class for static methods to call
};

CanmoreImageTransmitter::CanmoreImageTransmitter(int ifIndex, uint8_t clientId, unsigned int maxDim,
                                                 bool encodeGrayscale, uint8_t defaultQuality):
    Canmore::CANSocket(ifIndex),
    clientId(clientId), txImgSize_(maxDim, maxDim), maxDim_(maxDim), convertToGrayscale_(encodeGrayscale),
    streamQuality_(defaultQuality) {
    pollGroup_.addFd(*this);

    // Allocate space for the transmit buffer depending on the socket MTU
    txBuf_.resize(getMaxFrameSize());

    // Initialize the jpeg encoder
    err_ = new jpeg_error_mgr;
    info_ = new jpeg_compress_struct;
    info_->err = jpeg_std_error(err_);
    jpeg_create_compress(info_);

    // Configure the methods to write the JPEG
    assert(info_->dest == NULL);
    static_assert(offsetof(canmore_destination_mgr, pub) == 0, "Public reference must be at start of array");

    // Allocate destination memory owned by jpeg encoder
    canmore_destination_mgr *dest = (canmore_destination_mgr *) (*info_->mem->alloc_small)(
        (j_common_ptr) info_, JPOOL_PERMANENT, sizeof(canmore_destination_mgr));
    info_->dest = &dest->pub;

    dest->pub.init_destination = &libjpegInitDestination;
    dest->pub.empty_output_buffer = &libjpegEmptyOutputBuffer;
    dest->pub.term_destination = &libjpegTermDestination;
    dest->imageTx = this;

    // Configure the jpeg encoder
    info_->image_width = txImgSize_.width;
    info_->image_height = txImgSize_.height;
    if (convertToGrayscale_) {
        info_->input_components = 1;  // Grayscale will only have 1 channel
        info_->in_color_space = JCS_GRAYSCALE;
    }
    else {
        info_->input_components = 3;
        info_->in_color_space = JCS_RGB;
    }
    jpeg_set_defaults(info_);
    jpeg_set_quality(info_, streamQuality_, TRUE);

    // Register to receive command packets over the socket
    struct can_filter rfilter[] = { { .can_id = CANMORE_CAMERA_FEED_CALC_CTRL_ID(clientId),
                                      .can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CANMORE_CAMERA_FEED_STD_ID_MASK) } };
    setRxFilters(std::span { rfilter });
}

CanmoreImageTransmitter::~CanmoreImageTransmitter() {
    jpeg_destroy_compress(info_);
    delete info_;
    delete err_;
}

void CanmoreImageTransmitter::recomputeEncoderSize() {
    // Compute the transmitted image size
    int resizeWidth, resizeHeight;
    double aspectRatio = (double) expectedInputSize_.width / (double) expectedInputSize_.height;
    if (aspectRatio > 1) {
        resizeWidth = maxDim_;
        resizeHeight = maxDim_ / aspectRatio;
    }
    else {
        resizeHeight = maxDim_;
        resizeWidth = maxDim_ * aspectRatio;
    }
    txImgSize_ = cv::Size(resizeWidth, resizeHeight);

    info_->image_width = txImgSize_.width;
    info_->image_height = txImgSize_.height;
    jpeg_set_defaults(info_);
    jpeg_set_quality(info_, streamQuality_, TRUE);
}

void CanmoreImageTransmitter::transmitImage(cv::Mat &img) {
    // Process any pending CAN packets in case we got disabled before this was called
    while (pollGroup_.processEvent(0))
        ;

    if (!streamEnabled_) {
        // Don't transmit if we're disabled
        return;
    }

    // Convert image to grayscale
    cv::Mat imgToEncode;

    if (img.depth() != CV_8U) {
        throw std::logic_error("Invalid image depth");
    }
    if (img.size() != expectedInputSize_) {
        expectedInputSize_ = img.size();
        recomputeEncoderSize();
    }

    // Handle size conversion
    if (convertToGrayscale_) {
        if (img.channels() == 3) {
            cv::Mat largeGrayImg;
            cv::cvtColor(img, largeGrayImg, cv::COLOR_BGR2GRAY);
            cv::resize(largeGrayImg, imgToEncode, txImgSize_);
        }
        else if (img.channels() == 2) {
            // Already grayscale
            cv::resize(img, imgToEncode, txImgSize_);
        }
        else {
            throw std::logic_error("Invalid number of image dimensions: Expected either color or grayscale image");
        }
    }
    else {
        if (img.channels() != 3) {
            throw std::logic_error("Invalid number of image dimensions: Expected color image");
        }
        cv::resize(img, imgToEncode, txImgSize_);
    }

    // Populate every scanline for each row in the input gray image
    std::vector<JSAMPROW> scanlines(info_->image_height);
    for (int i = 0; i < scanlines.size(); i++) {
        scanlines.at(i) = imgToEncode.ptr(i, 0);
    }

    // Perform the compression
    jpeg_start_compress(info_, TRUE);
    JDIMENSION lineOutCnt = jpeg_write_scanlines(info_, scanlines.data(), scanlines.size());
    if (lineOutCnt != scanlines.size()) {
        throw std::runtime_error("Failed to encode all JPEG scanlines");
    }
    jpeg_finish_compress(info_);
}

void CanmoreImageTransmitter::libjpegInitDestination(j_compress_ptr cinfo) {
    // Safe to cast (we checked with the static assert in the constructor)
    canmore_destination_mgr *dest = (canmore_destination_mgr *) cinfo->dest;

    // Reset the buffer back to the start
    dest->pub.next_output_byte = dest->imageTx->txBuf_.data();
    dest->pub.free_in_buffer = dest->imageTx->txBuf_.size();

    // Tell the class that we're starting a new frame
    dest->imageTx->startNewImage();
}

boolean CanmoreImageTransmitter::libjpegEmptyOutputBuffer(j_compress_ptr cinfo) {
    // Safe to cast (we checked with the static assert in the constructor)
    canmore_destination_mgr *dest = (canmore_destination_mgr *) cinfo->dest;

    // Transmit the full buffer
    // The libjpeg-turbo library makes this assumption, so I'm assuming I can as well
    // If not, all of this logic gets SIGNIFICANTLY more complicated
    dest->imageTx->transmitNextImageFrame();

    // Reset the buffer back to the start
    dest->pub.next_output_byte = dest->imageTx->txBuf_.data();
    dest->pub.free_in_buffer = dest->imageTx->txBuf_.size();

    // Always successful (we won't overrun, since can transmit should block)
    return TRUE;
}

void CanmoreImageTransmitter::libjpegTermDestination(j_compress_ptr cinfo) {
    // Safe to cast (we checked with the static assert in the constructor)
    canmore_destination_mgr *dest = (canmore_destination_mgr *) cinfo->dest;

    // Get count of remaining data in buffer
    auto &txBuf = dest->imageTx->txBuf_;
    size_t pendingCnt = txBuf.size() - dest->pub.free_in_buffer;

    // Transmit if any remaining bytes in the buffer
    if (pendingCnt > 0) {
        dest->imageTx->transmitNextImageFrame(pendingCnt);
    }

    // Finish the image transmission
    dest->imageTx->finishImage();
}

void CanmoreImageTransmitter::handleFrame(canid_t can_id, const std::span<const uint8_t> &data) {
    (void) can_id;

    canmore_camera_feed_cmd_t cmd;
    if (data.size() > sizeof(cmd)) {
        // Don't process if we'll overflow cmd
        return;
    }

    std::copy(data.begin(), data.end(), cmd.data);

    if (cmd.pkt.cmd == CANMORE_CAMERA_FEED_CMD_ENABLE) {
        streamEnabled_ = !!cmd.pkt.data.enable;
    }
    else if (cmd.pkt.cmd == CANMORE_CAMERA_FEED_CMD_STREAM_ID) {
        streamId_ = cmd.pkt.data.stream_id;
    }
    else if (cmd.pkt.cmd == CANMORE_CAMERA_FEED_CMD_QUALITY) {
        uint8_t quality = cmd.pkt.data.quality;
        if (quality > 100)
            return;
        streamQuality_ = quality;
        jpeg_set_quality(info_, quality, TRUE);
    }
    else if (cmd.pkt.cmd == CANMORE_CAMERA_FEED_CMD_MAX_DIMENSION) {
        maxDim_ = cmd.pkt.data.max_dimension;
        recomputeEncoderSize();
    }
    else if (cmd.pkt.cmd == CANMORE_CAMERA_FEED_CMD_KEYPRESS) {
        pendingKeypress_ = cmd.pkt.data.keypress;
    }
}

void CanmoreImageTransmitter::startNewImage() {
    // Reset transmit state variables
    nextFrameIdx_ = 0;
    totalTxLen_ = 0;
    crc32State_ = CRC32_INITIAL_VALUE;
}

void CanmoreImageTransmitter::transmitNextImageFrame(size_t fragmentLen) {
    // Compute the frame id (extended to hold the next frame id)
    canid_t id = CAN_EFF_FLAG | CANMORE_CAMERA_FEED_CALC_FRAME_ID(clientId, nextFrameIdx_);
    nextFrameIdx_++;

    // Handle fragment size
    size_t frameSize;
    if (!fragmentLen) {
        // If no fragment length is provided, transmit the entire buffer
        fragmentLen = txBuf_.size();
        frameSize = fragmentLen;
    }
    else {
        // Fragment length provided, make sure we can pack it into a CAN frame
        // In case we're using CAN FD, we need to check that we can fit the last frame into the buffer
        frameSize = canmore_fd_dlc2len(canmore_fd_len2dlc(fragmentLen));
        assert(frameSize <= txBuf_.size());

        if (frameSize != fragmentLen) {
            assert(frameSize > fragmentLen);
            // Size didn't match, we need to fill the rest buffer with 0s so we don't transmit random garbage
            std::fill(txBuf_.begin() + fragmentLen, txBuf_.begin() + frameSize, 0);
        }
    }

    // Update transmission state and send the frame
    totalTxLen_ += fragmentLen;
    crc32State_ = crc32_update(txBuf_.data(), fragmentLen, crc32State_);
    transmitFrame(id, std::span<const uint8_t> { txBuf_.data(), frameSize });
}

void CanmoreImageTransmitter::finishImage() {
    // Send the final frame (standard frame holding length and computed CRC32 value)
    canmore_camera_feed_last_frame_t lastFrame;

    static_assert(sizeof(lastFrame) == 8, "Struct did not pack properly");

    lastFrame.pkt.len = totalTxLen_;
    lastFrame.pkt.crc32 = crc32State_;

    canid_t id = CANMORE_CAMERA_FEED_CALC_LAST_FRAME_ID(clientId);

    transmitFrame(id, std::span { lastFrame.data });
}
