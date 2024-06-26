#pragma once

#include "RemoteTTYServer.hpp"
#include "canmore_cpp/CANSocket.hpp"
#include "canmore_cpp/RegMappedServer.hpp"

#include <map>
#include <signal.h>
#include <string>
#include <sys/types.h>

uint64_t discoverSerialNumber(void);

class HeartbeatTransmitter {
public:
    HeartbeatTransmitter(int ifIndex, int clientId);
    ~HeartbeatTransmitter();

    void start();

    const int ifIndex;
    const int clientId;

private:
    int socketFd;
};

class CanmoreLinuxServer : public Canmore::RegMappedCANServer {
public:
    CanmoreLinuxServer(int ifIndex, uint8_t clientId);

    bool stopRequested() { return shouldStop_; }
    void getTtyInitialConfig(std::string &termEnv, uint16_t &initialRows, uint16_t &initialCols, std::string &cmd,
                             std::string &workingDir);
    bool getTtyEnabled() { return remoteTtyEnabled_; }
    void notifyTtyShutdown() { remoteTtyEnabled_ = false; }

    // This is part of this class, since we'll always have a socket here
    // However, it sends over the Remote TTY protocol to disconnect any still active clients
    // Prevents issues where this program restarts, but starts with the remote tty disabled, but canmore cli is still
    // in remotesh mode (and all keyboard input is redirected)
    // Safe to be called in destructors and signal handlers
    void forceTTYdisconnect() noexcept;

private:
    bool shouldStop_ = false;
    bool remoteTtyEnabled_ = false;
    bool current_operation_ = false;
    uint32_t windowSzReg_ = 0;
    uint32_t filenameLengthReg_ = 0;
    uint32_t dataLengthReg_ = 0;
    uint32_t crc32Reg_ = 0;
    uint32_t clearFileReg_ = 0;
    uint32_t readOffsetReg_ = 0;
    uint32_t fileModeReg_ = 0;
    uint32_t writeStatusReg_ = 0;
    uint32_t currentFileCrc_ = 0xFFFFFFFF;
    uint32_t runCmdFromUploadDir_ = 0;
    std::vector<uint8_t> termStrBuf_;
    std::vector<uint8_t> cmdBuf_;
    std::vector<uint8_t> fileBuf_;
    std::string file_pwd_;
    uint8_t readBuf_[REG_MAPPED_PAGE_SIZE];

    bool restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
    bool enableTtyCb(uint16_t addr, bool is_write, uint32_t *data_ptr);
    void doFileWrite();
    void doFileRead();
    void doSetFileMode();
    void doGetFileMode();
    void doGetFileLen();
    void doCheckCrc();
    void doCd();
    void doLs();
    void doPwd();
    bool triggerFileOperation(uint16_t addr, bool is_write, uint32_t *data_ptr);
    void reportDeviceError(const char *error);
    std::string readFileNameFromBuf();
    bool readFiletoString(const std::string &filename, std::string &contents);
    bool writeStringToFile(const std::string &filename, const std::string &contents);
    std::string joinPaths(const std::string &path1, const std::string &path2);
    std::string getHome();
};

class CanmoreTTYServer : public Canmore::RemoteTTYServerEventHandler, public Canmore::PollFDHandler {
public:
    CanmoreTTYServer(int ifIndex, uint8_t clientId, const std::string &termEnv = "", uint16_t initialRows = 0,
                     uint16_t initialCols = 0, const std::string &cmd = "", const std::string &workingDir = "");
    ~CanmoreTTYServer();

    // Override to configure PollFDHandler
    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;

    /**
     * @brief Reports if the TTYServer has terminated
     *
     * @param inError Sets this reference to report if the server has terminated due to an error
     * @return true The server has terminated. Destroy the object to ensure proper cleanup
     * @return false The server is still running
     */
    bool isTerminated(bool &inError) { return canmoreServer_.isDisconnected(inError); }

protected:
    // Handler for PollFDHandler
    void handleEvent(const pollfd &fd) override;

    // Handlers for RemoteTTYServerEventHandler
    void handleStdin(const std::span<const uint8_t> &data) override;
    void handleWindowSize(uint16_t rows, uint16_t cols) override;
    void handleStdioReady() override;

private:
    Canmore::RemoteTTYServer canmoreServer_;
    int ptyMasterFd_ = -1;
    int sigchldEventFd_ = -1;
    volatile pid_t childPid_ = -1;
    std::shared_ptr<Canmore::PollFDDescriptor> ptyDescriptor_;
    std::shared_ptr<Canmore::PollFDDescriptor> sigchldEventDescriptor_;

    // Reads the sigchldEventFd_ to see if the given child died normally or due to an error. This should ONLY be
    // called if childPid_ = -1, or else this will throw an error (as there won't be anything in the eventfd)
    bool childDiedInErr();
    bool sigchldEventFdRead_ = false;
    bool sigchldEventReportedErr_ = false;

    // Disconnects the client due to a pty error is received, reporting the error
    // If the child died, however, this will disconnect reporting if the child terminated in error
    // This is needed since the child can die at any time (including during a syscall). To avoid erronious errors, any
    // errors which can occur from the pty being disconnected should call this, which will set the error accordingly
    void disconnectFromPtyErr();

    // ========================================
    // Static Signal Handler Routines
    // ========================================

    // This installs the sigchld handler to catch when the child goes down
    // Needed since the traditional waitpid can't handle processes which survive past a SIGHUP, but die after, leaving
    // a zombie. Because the owner class has already been destroyed, nothing will wait that process. Instead, this will
    // collect all children, and report them if the owner instance is still alive, and discard the status data if the
    // owner instance has died

    static std::map<pid_t, CanmoreTTYServer *> activeChildren;
    static bool sigchldHandlerInstalled;
    static void sigchldHandler(int signum, siginfo_t *info, void *ucontext);
    static void sigchldInstallHandler();
    static void sigchldBlock();
    static void sigchldUnblock();
};

class SystemdNotifier {
public:
    SystemdNotifier() { reportStatus("Initialized Systemd Notifier"); }

    void reportReady() { notifyState("READY=1"); }
    void reportStopping() { notifyState("STOPPING=1"); }
    void reportStatus(const std::string &status) { notifyState("STATUS=" + status); }
    void feedWatchdog() { notifyState("WATCHDOG=1"); }
    void triggerWatchdogRestart() { notifyState("WATCHDOG=trigger"); }

private:
    void notifyState(const std::string &state);
};
