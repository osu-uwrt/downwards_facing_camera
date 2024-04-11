#pragma once
#include "canmore_cpp/CANSocket.hpp"

#include <opencv2/opencv.hpp>

class CanmoreImageTransmitter : protected Canmore::CANSocket {
public:
    /**
     * @brief Construct a new Canmore Image Transmitter object
     *
     * @param ifIndex The interface index to bind to
     * @param clientId The client ID this image transmitter belongs to
     * @param maxDim The maximum dimension (of both height and width) of the resulting output image
     * @param encodeGrayscale Converts the image to grayscale before encoding to reduce image size
     * @param defaultQuality The default quality for JPEG encoding (can be overridden by client)
     */
    CanmoreImageTransmitter(int ifIndex, uint8_t clientId, unsigned int maxDim = 240, bool encodeGrayscale = false,
                            uint8_t defaultQuality = 30);
    ~CanmoreImageTransmitter();

    /**
     * @brief Transmits a new image
     *
     * @param img Input image in BGR format (must be of size inputSize passed to the constructor)
     */
    void transmitImage(cv::Mat &img);

    /**
     * @brief Checks if transmit is enabled, and if so, sets the streamIdOut to the currently selected stream id
     *
     * @param streamIdOut Written with the currently selected stream id if transmit is enabled
     * @return true Transmit image is enabled (frames will be transmitted over the bus)
     * @return false Image transmission is disabled
     */
    bool transmitEnabled(uint8_t &streamIdOut) {
        // Process any pending CAN packets
        while (pollGroup_.processEvent(0))
            ;

        // Report the current state
        if (streamEnabled_) {
            streamIdOut = streamId_;
            return true;
        }
        else {
            return false;
        }
    }

    const uint8_t clientId;

protected:
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;

private:
    void recomputeEncoderSize(cv::Size inputSize);

    static void libjpegInitDestination(struct jpeg_compress_struct *cinfo);
    static int libjpegEmptyOutputBuffer(struct jpeg_compress_struct *cinfo);
    static void libjpegTermDestination(struct jpeg_compress_struct *cinfo);

    void startNewImage();
    void transmitNextImageFrame(size_t fragmentLen = 0);
    void finishImage();

    // ===== Image Compression State =====

    // Holds the expected size of input images
    cv::Size expectedInputSize_;
    // Holds the computed size for the images to be transmitted after compression
    cv::Size txImgSize_;
    // The maximum dimension for compressed images
    unsigned int maxDim_;
    // If true, the image is converted to grayscale first
    bool convertToGrayscale_;

    // ===== JPEG Encoder State =====

    // Jpeg compressor state
    struct jpeg_compress_struct *info_;
    // Error handling state for jpeg compression
    struct jpeg_error_mgr *err_;

    // ===== Frame Transmission State =====

    // Temporary buffer for holding bytes to be transmitted in the next frame
    std::vector<uint8_t> txBuf_;
    // Holds index of the next frame to be sent
    uint32_t nextFrameIdx_;
    // Holds the size of the complete data transmission
    size_t totalTxLen_;
    // Holds the computed CRC32 of the image
    uint32_t crc32State_;

    // ===== CAM Command Processing =====

    // A poll group which will be executed whenever we want to process any pending packets
    Canmore::PollGroup pollGroup_;
    bool streamEnabled_ = false;
    uint8_t streamId_ = 0;
    uint8_t streamQuality_;
};
