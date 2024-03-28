#include "DFCDaemon.hpp"

#if HAS_SYSTEMD
#include <systemd/sd-daemon.h>
#endif

void SystemdNotifier::notifyState(const std::string &state) {
    (void) state;

#if HAS_SYSTEMD
    sd_notify(0, state.c_str());
#endif
}
