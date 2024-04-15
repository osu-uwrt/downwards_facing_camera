#include "canmore_cpp/MsgClient.hpp"
#include "micro_ros_transport.h"

#include "canmore/protocol.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <chrono>
#include <iostream>
#include <memory>

// ========================================
// C++ Transport Wrapper Classes
// ========================================

class MicroROSClient : public Canmore::ClientMsgHandler {
public:
    MicroROSClient(unsigned int ifIndex, uint8_t clientId): msgClient(ifIndex, clientId, *this) {
        pollGroup.addFd(msgClient);
    }

    void writePacket(const uint8_t *buf, size_t len) {
        if (len > CANMORE_MAX_MSG_LENGTH) {
            throw std::runtime_error("Cannot transmit message greater than max length");
        }
        msgClient.transmitMessage(CANMORE_MSG_SUBTYPE_XRCE_DDS, std::span<const uint8_t>(buf, len));
    }

    size_t tryReadPacket(uint8_t *bufOut, size_t max_len, int timeoutMs) {
        assert(copyBuf == NULL);
        copyBuf = new std::span<uint8_t> { bufOut, max_len };
        copyBufWriteLen = 0;

        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::milliseconds ms;

        const auto start = Clock::now();

        while (copyBufWriteLen == 0) {
            const auto now = Clock::now();
            const auto elapsedTime = std::chrono::duration_cast<ms>(now - start);
            const int64_t remainingMs = timeoutMs - elapsedTime.count();

            if (remainingMs < 0) {
                break;
            }
            pollGroup.processEvent(remainingMs);
        }

        delete copyBuf;
        copyBuf = NULL;
        return copyBufWriteLen;
    }

protected:
    void handleMessage(uint8_t subtype, std::span<const uint8_t> data) override {
        if (subtype != CANMORE_MSG_SUBTYPE_XRCE_DDS) {
            return;
        }
        size_t copyLen = data.size();
        if (copyBuf->size() < copyLen) {
            copyLen = copyBuf->size();
        }
        std::copy(data.begin(), data.begin() + copyLen, copyBuf->begin());
        copyBufWriteLen = copyLen;
    }

    void handleDecodeError(unsigned int errorCode) override {}

private:
    size_t copyBufWriteLen = 0;
    std::span<uint8_t> *copyBuf = NULL;

    Canmore::PollGroup pollGroup;
    Canmore::MsgClient msgClient;
};

struct ClientPersistentWrapper {
    ClientPersistentWrapper(int ifIndex, uint8_t clientId): ifIndex(ifIndex), clientId(clientId) {}

    std::unique_ptr<MicroROSClient> client;
    unsigned int ifIndex;
    uint8_t clientId;
};

// ========================================
// C Transport Bindings
// ========================================

static bool transport_open(struct uxrCustomTransport *transport) {
    ClientPersistentWrapper *wrapper = reinterpret_cast<ClientPersistentWrapper *>(transport->args);
    try {
        wrapper->client = std::make_unique<MicroROSClient>(wrapper->ifIndex, wrapper->clientId);
        return true;
    } catch (std::exception &e) {
        std::cerr << "Error opening transport: " << e.what() << std::endl;
        return false;
    }
}

static bool transport_close(struct uxrCustomTransport *transport) {
    ClientPersistentWrapper *wrapper = reinterpret_cast<ClientPersistentWrapper *>(transport->args);

    wrapper->client.reset();
    return true;
}

static size_t transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
    ClientPersistentWrapper *wrapper = reinterpret_cast<ClientPersistentWrapper *>(transport->args);

    try {
        wrapper->client->writePacket(buf, len);
        return len;
    } catch (...) {
        *errcode = 1;
        return 0;
    }
}

static size_t transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                             uint8_t *errcode) {
    ClientPersistentWrapper *wrapper = reinterpret_cast<ClientPersistentWrapper *>(transport->args);

    try {
        return wrapper->client->tryReadPacket(buf, len, timeout);
    } catch (...) {
        *errcode = 1;
        return 0;
    }
}

// ========================================
// RMW Error Handling Code
// ========================================

const char *const entity_lookup_table[] = {
    "RMW_UROS_ERROR_ON_UNKNOWN",      "RMW_UROS_ERROR_ON_NODE",
    "RMW_UROS_ERROR_ON_SERVICE",      "RMW_UROS_ERROR_ON_CLIENT",
    "RMW_UROS_ERROR_ON_SUBSCRIPTION", "RMW_UROS_ERROR_ON_PUBLISHER",
    "RMW_UROS_ERROR_ON_GRAPH",        "RMW_UROS_ERROR_ON_GUARD_CONDITION",
    "RMW_UROS_ERROR_ON_TOPIC",
};
const char *const source_lookup_table[] = {
    "RMW_UROS_ERROR_ENTITY_CREATION", "RMW_UROS_ERROR_ENTITY_DESTRUCTION",    "RMW_UROS_ERROR_CHECK",
    "RMW_UROS_ERROR_NOT_IMPLEMENTED", "RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION",
};

#define lookup_string_enum(value, list) ((value < sizeof(list) / sizeof(*list)) ? list[value] : "Out-of-Bounds")
#define lookup_entity_enum(value) lookup_string_enum(value, entity_lookup_table)
#define lookup_source_enum(value) lookup_string_enum(value, source_lookup_table)

static void rmw_error_cb(const rmw_uros_error_entity_type_t entity, const rmw_uros_error_source_t source,
                         const rmw_uros_error_context_t context, const char *file, const int line) {
    fprintf(stderr, "\033[1;33mRMW UROS Error:\n\tEntity: %s\n\tSource: %s\n\tDesc: %s\n\tLocation: %s:%d\n\033[0m",
            lookup_entity_enum(entity), lookup_source_enum(source), context.description, file, line);
}

// ========================================
// Exported Transport Init
// ========================================

void micro_ros_transport_init(unsigned int ifIndex, uint8_t clientId) {
    static ClientPersistentWrapper persistentWrapper(ifIndex, clientId);

    rmw_uros_set_error_handling_callback(rmw_error_cb);

    if (rmw_uros_set_custom_transport(false, &persistentWrapper, &transport_open, &transport_close, &transport_write,
                                      &transport_read) != RMW_RET_OK) {
        throw std::runtime_error("Failed to intialize ROS custom transport");
    }
}
