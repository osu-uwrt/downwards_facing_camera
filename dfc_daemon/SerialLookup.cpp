#include "DFCDaemon.hpp"

#include <dirent.h>
#include <fstream>
#include <set>
#include <stdint.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>

static inline uint8_t hexCharToNibble(char val) {
    if (val >= '0' && val <= '9') {
        return val - '0';
    }
    else if (val >= 'a' && val <= 'f') {
        return (val - 'a') + 0xA;
    }
    else if (val <= 'A' && val <= 'F') {
        return (val - 'A') + 0xA;
    }
    else {
        return 0xFF;
    }
}

uint64_t discoverSerialNumber(void) {
    // Define serial number (with initial value 0 to report an error retreiving serial number)
    union serial_num {
        uint8_t byte[8];
        uint64_t word;
    } serial = { .word = 0 };

    std::string netInfoDir = "/sys/class/net/";

    DIR *dp;
    struct dirent64 *ep;
    dp = opendir(netInfoDir.c_str());
    if (dp == NULL) {
        // /sys/class/net doesn't exist, we can't determine the serial number
        return 0;
    }

    // Try all interfaces to find physical interfaces with a MAC address
    std::set<std::string> availableInterfaces;
    while ((ep = readdir64(dp)) != NULL) {
        if (ep->d_type != DT_LNK) {
            continue;
        }

        std::string interfacePath = netInfoDir + ep->d_name;
        char *actualPathCstr = realpath(interfacePath.c_str(), NULL);
        if (actualPathCstr == NULL) {
            perror("realpath");
            continue;
        }

        std::string actualPath = actualPathCstr;
        free(actualPathCstr);

        // Only match /sys/devices that aren't virtual
        if (actualPath.rfind("/sys/devices/", 0) != 0 || actualPath.rfind("/sys/devices/virtual/", 0) == 0) {
            continue;
        }

        // Now retreive the MAC address from this interface
        actualPath += "/address";

        if (access(actualPath.c_str(), F_OK) != 0) {
            // Couldn't find the file, don't try to run it
            continue;
        }

        availableInterfaces.emplace(actualPath);
    }
    (void) closedir(dp);

    // Now try all the interfaces in sorted order
    // This makes it deterministic (so long as network interfaces remain consistent)
    for (const std::string &addrPath : availableInterfaces) {
        try {
            std::ifstream addrStream(addrPath);
            std::string macAddr;
            addrStream >> macAddr;
            if (macAddr.size() != 17) {
                // Expect MAC address to be in format XX:XX:XX:XX:XX:XX
                // Anything else we won't parse correctly
                continue;
            }

            bool valid = true;
            uint8_t macAddrDecoded[6];
            auto itr = macAddr.begin();
            for (int i = 0; i < 6; i++) {
                if (i != 0) {
                    if (*itr++ != ':') {
                        valid = false;
                        break;
                    }
                }

                uint8_t highNibble = hexCharToNibble(*itr++);
                uint8_t lowNibble = hexCharToNibble(*itr++);
                if (highNibble == 0xFF || lowNibble == 0xFF) {
                    valid = false;
                    break;
                }
                macAddrDecoded[i] = (highNibble << 4) | lowNibble;
            }
            if (!valid) {
                continue;
            }

            serial.byte[0] = 'N';
            serial.byte[1] = 'M';
            for (int i = 0; i < 6; i++) {
                serial.byte[2 + i] = macAddrDecoded[i];
            }

            // We got a serial number, break
            break;
        } catch (...) {
            continue;
        }
    }

    return serial.word;
}
