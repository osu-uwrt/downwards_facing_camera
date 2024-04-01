#pragma once
#include "canmore_cpp/CANSocket.hpp"

#include <vector>

class CanmoreImageReceiverHandler {
protected:
    friend class CanmoreImageReceiver;
    virtual void handleJpeg(const std::span<const uint8_t> &data) = 0;
};

class CanmoreImageReceiver : public Canmore::CANSocket {
public:
    /**
     * @brief Construct a new Canmore Image Receiver object
     *
     * @param ifIndex The interface index to bind to
     * @param clientId The client ID this image transmitter belongs to
     * @param handler The handler to call with the JPEG upon successful reception
     */
    CanmoreImageReceiver(int ifIndex, uint8_t clientId, CanmoreImageReceiverHandler &handler);

    /**
     * @brief Enables/Disables the remote stream
     *
     * @param enable Set to true to enable the remote stream, false to disable
     */
    void setStreamEnabled(bool enable);

    /**
     * @brief Requests the given stream id from the client
     *
     * @param streamId The stream id to request (defined by application)
     */
    void setStreamId(uint8_t streamId);

    /**
     * @brief Sets the stream quality for the requested stream
     *
     * @param quality The JPEG compression quality (must be 0-100)
     */
    void setStreamQuality(uint8_t quality);

    const uint8_t clientId;

protected:
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) override;

private:
    CanmoreImageReceiverHandler &handler_;
    bool decodeStop_ = true;
    std::vector<uint8_t> rxBuf_;
    uint32_t nextFrameIdx_;
};
