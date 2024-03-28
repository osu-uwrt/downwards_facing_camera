#include "DFCDaemon.hpp"

#include <pty.h>
#include <pwd.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <unistd.h>

// ========================================
// Static Signal Handler Implementation
// ========================================

bool CanmoreTTYServer::sigchldHandlerInstalled = false;
std::map<pid_t, CanmoreTTYServer *> CanmoreTTYServer::activeChildren = {};

void CanmoreTTYServer::sigchldHandler(int signum, siginfo_t *info, void *ucontext) {
    if (signum != SIGCHLD) {
        return;
    }

    auto itr = activeChildren.find(info->si_pid);
    if (itr == activeChildren.end()) {
        // Couldn't find the pid in the list, ignore the info it must have been from a hungup instance
        return;
    }

    itr->second->childPid_ = -1;

    // Report that the child died in error if it died from a reason other than an exit, or if it did exit but the exit
    // code wasn't 0
    bool diedInError = (info->si_code != CLD_EXITED || info->si_status != 0);

    // Notify of termination using eventfd
    // A counter of 1 means normal termination, anything else is abnormal
    uint64_t count = (diedInError ? 2 : 1);
    if (write(itr->second->sigchldEventFd_, &count, sizeof(count)) != sizeof(count)) {
        const char errmsg[] = "Failed to write eventfd notifying child death\n";
        write(STDERR_FILENO, errmsg, sizeof(errmsg) - 1);
        abort();
    }

    // Remove the active child from the list
    // Although you can get away without this, in the event the PID is taken up again by another child spawning before
    // the destructor is called on the old server, you can get into some nasty race conditions
    activeChildren.erase(itr);
}

void CanmoreTTYServer::sigchldInstallHandler() {
    if (sigchldHandlerInstalled) {
        return;
    }

    // This SHOULD NOT FAIL. The only way this fails is if we are being dumb (see man sigaction)
    // If it does, then trying to handle graceful cleanup is a lot more complex and would probably produce worse code
    // Just abort on failure, as this is a problem with us

    // Define signal handler
    struct sigaction act = {};
    act.sa_sigaction = &CanmoreTTYServer::sigchldHandler;
    // Only report child death, remove child waiting (to handle processes surviving NOHUP), and we're a fancy handler
    act.sa_flags = SA_NOCLDSTOP | SA_NOCLDWAIT | SA_SIGINFO;

    // Install signal handler
    struct sigaction oldact;
    if (sigaction(SIGCHLD, &act, &oldact) || (oldact.sa_flags & SA_SIGINFO) || oldact.sa_handler != SIG_DFL) {
        const char errmsg[] = "sigaction SIGCHLD failed\n";
        write(STDERR_FILENO, errmsg, sizeof(errmsg) - 1);
        abort();
    }

    sigchldHandlerInstalled = true;
}

void CanmoreTTYServer::sigchldBlock() {
    // Same as above, THIS SHOULD NOT FAIL

    // Define the set
    sigset_t set, oldset;
    sigemptyset(&set);
    sigaddset(&set, SIGCHLD);

    // Block it, and verify we aren't recursively called
    if (sigprocmask(SIG_BLOCK, &set, &oldset) || sigismember(&oldset, SIGCHLD)) {
        const char errmsg[] = "sigprocmask block SIGCHLD failed\n";
        write(STDERR_FILENO, errmsg, sizeof(errmsg) - 1);
        abort();
    }
}

void CanmoreTTYServer::sigchldUnblock() {
    // Same as above, THIS SHOULD NOT FAIL

    // Define the set
    sigset_t set, oldset;
    sigemptyset(&set);
    sigaddset(&set, SIGCHLD);

    // Block it, and verify we aren't recursively called
    if (sigprocmask(SIG_UNBLOCK, &set, &oldset) || !sigismember(&oldset, SIGCHLD)) {
        const char errmsg[] = "sigprocmask unblock SIGCHLD failed\n";
        write(STDERR_FILENO, errmsg, sizeof(errmsg) - 1);
        abort();
    }
}

// ========================================
// Class Implementation
// ========================================

#define cerrchk(func)                                                                                                  \
    if (func) {                                                                                                        \
        perror(#func);                                                                                                 \
        _exit(1);                                                                                                      \
    }

CanmoreTTYServer::CanmoreTTYServer(int ifIndex, uint8_t clientId, const std::string &termEnv, uint16_t initialRows,
                                   uint16_t initialCols, const std::string &cmd, const std::string &workingDir):
    canmoreServer_(*this, ifIndex, clientId) {
    // Open the eventfd for reporting when the child dies
    sigchldEventFd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);

    if (sigchldEventFd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "eventfd");
    }

    // Set up the child handler if needed, then block sigchild signals during startup to prevent race conditions
    sigchldInstallHandler();
    sigchldBlock();

    // Spawn the child
    struct winsize winp = {};
    winp.ws_row = initialRows;
    winp.ws_col = initialCols;
    childPid_ = forkpty(&ptyMasterFd_, NULL, NULL, &winp);

    if (childPid_ < 0) {
        sigchldUnblock();
        close(sigchldEventFd_);
        throw std::system_error(errno, std::generic_category(), "forkpty");
    }

    if (childPid_ == 0) {
        // We are the child

        // Load information about who we're running as so we can spawn them a shell
        uid_t ruid, euid, suid;
        gid_t rgid, egid, sgid;
        cerrchk(getresuid(&ruid, &euid, &suid));
        cerrchk(getresgid(&rgid, &egid, &sgid));

        // Clear any special privileges if the process has it back to the real uid/gid
        if (ruid != euid || ruid != suid) {
            cerrchk(setresuid(ruid, ruid, ruid));
        }
        if (rgid != egid || rgid != sgid) {
            cerrchk(setresgid(rgid, rgid, rgid));
        }

        // Look up info about the user for this process
        struct passwd *pwdent;
        do {
            pwdent = getpwent();
            if (pwdent == NULL) {
                const char errmsg[] = "Could not locate user entry in passwd database\n";
                write(STDERR_FILENO, errmsg, strlen(errmsg));
            }
        } while (pwdent->pw_uid != ruid);
        char *username = strdup(pwdent->pw_name);
        char *homedir = strdup(pwdent->pw_dir);
        char *loginshell = strdup(pwdent->pw_shell);
        endpwent();

        // Clear the system environment to a fresh one
        cerrchk(clearenv());

        // Go to working dir if specified
        if (workingDir.empty() || chdir(workingDir.c_str())) {
            // Fall back to home directory if not
            if (chdir(homedir)) {
                // If that fails (user's home directory), go to the root directory
                cerrchk(chdir("/"));
            }
        }

        // Set the user environment variables
        cerrchk(setenv("USER", username, true));
        cerrchk(setenv("USERNAME", username, true));
        cerrchk(setenv("HOME", homedir, true));
        cerrchk(setenv("SHELL", loginshell, true));
        cerrchk(setenv("LANG", "en_US.UTF-8", true));

        // Set the terminal environment variable
        if (!termEnv.empty()) {
            cerrchk(setenv("TERM", termEnv.c_str(), true));
        }

        // Determine arg 0 (will either be normal or login shell, depending if running a command)
        char *shellName = basename(loginshell);
        char shellArg0[strlen(shellName) + 2];
        if (cmd.empty()) {
            // No command, full login shell
            shellArg0[0] = '-';
            shellArg0[1] = '\0';
        }
        else {
            // Running command, just spawn a standard shell (no leading - on command name)
            shellArg0[0] = '\0';
        }
        strncat(&shellArg0[1], shellName, sizeof(shellArg0) - 2);
        shellArg0[sizeof(shellArg0) - 1] = 0;

        if (!cmd.empty()) {
            execl(loginshell, shellArg0, "-c", cmd.c_str(), NULL);
        }
        else {
            execl(loginshell, shellArg0, NULL);
        }

        perror("execl");
        _exit(1);
    }

    // Add the childpid to the active children map
    activeChildren.emplace(childPid_, this);

    // It should now be safe to re-enable the sigchld
    sigchldUnblock();

    ptyDescriptor_ = Canmore::PollFDDescriptor::create(*this, ptyMasterFd_, POLLIN);
    sigchldEventDescriptor_ = Canmore::PollFDDescriptor::create(*this, sigchldEventFd_, POLLIN);
}

CanmoreTTYServer::~CanmoreTTYServer() {
    // Clean up the child
    // Need to temporarily disable sigchld to prevent race conditions
    sigchldBlock();
    if (childPid_ > 0) {
        pid_t pidCapture = childPid_;
        auto itr = activeChildren.find(pidCapture);
        if (itr != activeChildren.end()) {
            activeChildren.erase(itr);
        }
    }
    sigchldUnblock();

    // This should send the SIGHUP to the process since we close the controlling terminal
    if (ptyMasterFd_ >= 0) {
        close(ptyMasterFd_);
    }

    if (sigchldEventFd_ >= 0) {
        close(sigchldEventFd_);
    }
}

void CanmoreTTYServer::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    canmoreServer_.populateFds(descriptors);
    descriptors.push_back(ptyDescriptor_);
}

void CanmoreTTYServer::handleStdin(const std::span<const uint8_t> &data) {
    // No point in writing, since we just disconnected so we're about to send a SIGHUP
    if (canmoreServer_.isDisconnected())
        return;

    // Write data out to the pty
    int rc = write(ptyMasterFd_, data.data(), data.size_bytes());
    if (rc < 0 || (size_t) rc != data.size_bytes()) {
        disconnectFromPtyErr();
    }
}

void CanmoreTTYServer::handleEvent(const pollfd &fd) {
    if (fd.fd == ptyMasterFd_) {
        if (fd.events & POLLIN) {
            // If we can't write, disable the poll event until we're notified it's ready again
            if (!canmoreServer_.stdioCanWrite()) {
                ptyDescriptor_->setEnabled(false);
                canmoreServer_.stdioNotifyWhenReady();
                return;
            }

            // Get the data from the fd
            std::vector<uint8_t> buf(canmoreServer_.getMaxFrameSize());
            int rc = read(ptyMasterFd_, buf.data(), buf.size());
            if (rc <= 0) {
                // Since we just disconnected, no use in polling for more data
                // Just disable pty descriptor and wait for the main function to destroy this instance
                disconnectFromPtyErr();
                ptyDescriptor_->setEnabled(false);
                return;
            }
            buf.resize(rc);

            // Forward it - note pty doesn't support stderr :(
            // Wish I knew that before writing the remotettyserver but ¯\_(ツ)_/¯
            canmoreServer_.stdoutWrite(buf);
        }
        else {
            canmoreServer_.disconnect(childDiedInErr());
            // Only other poll events we get are error events on the tty
            // Just disconnect and report an error
            disconnectFromPtyErr();
        }
    }
    else if (fd.fd == sigchldEventFd_) {
        // Can only disconnect once, ignore all future things in the eventfd (but they shouldn't occur)
        sigchldEventDescriptor_->setEnabled(false);
    }
    else {
        throw std::logic_error("Unexpected fd in canmore tty server handleEvent: " + std::to_string(fd.fd));
    }
}

void CanmoreTTYServer::handleWindowSize(uint16_t rows, uint16_t cols) {
    // No use trying to write to the window size register if we disconnected, since we might have lost the pty
    // And, if it's disconnected, it's about to be killed soon
    if (canmoreServer_.isDisconnected()) {
        return;
    }

    struct winsize newSize = {};
    newSize.ws_col = cols;
    newSize.ws_row = rows;
    if (ioctl(ptyMasterFd_, TIOCSWINSZ, &newSize)) {
        // The tty must have died, just report the error and disconect
        disconnectFromPtyErr();
    }
}

void CanmoreTTYServer::handleStdioReady() {
    ptyDescriptor_->setEnabled(true);
}

bool CanmoreTTYServer::childDiedInErr() {
    if (sigchldEventFdRead_) {
        return sigchldEventReportedErr_;
    }

    uint64_t count;
    if (read(sigchldEventFd_, &count, sizeof(count)) != sizeof(count)) {
        throw std::system_error(errno, std::generic_category(), "eventfd read");
    }

    sigchldEventReportedErr_ = count > 1;
    sigchldEventFdRead_ = true;

    return sigchldEventReportedErr_;
}

void CanmoreTTYServer::disconnectFromPtyErr() {
    bool isErr = true;
    if (childPid_ < 0) {
        isErr = childDiedInErr();
    }
    canmoreServer_.disconnect(isErr);
}
