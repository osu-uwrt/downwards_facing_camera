#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void micro_ros_transport_init(unsigned int ifIndex, uint8_t clientId);

#ifdef __cplusplus
}
#endif
