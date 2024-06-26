#include "DFCDaemon.hpp"

#include "canmore/crc32.h"
#include "canmore/reg_mapped/interface/linux.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string.h>
#include <sys/stat.h>

#define bind_reg_cb(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)

const char version_str[] = "dfc_daemon (" __DATE__ " " __TIME__ ")";

static inline bool isAbsolutePath(const std::string &str) {
    return str.size() != 0 && str.at(0) == '/';
}

CanmoreLinuxServer::CanmoreLinuxServer(int ifIndex, uint8_t clientId):
    Canmore::RegMappedCANServer(ifIndex, clientId, CANMORE_CHAN_CONTROL_INTERFACE,
                                CANMORE_CONTROL_INTERFACE_MODE_LINUX) {
    // Define general control page
    auto gen_control = Canmore::RegMappedRegisterPage::create();
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_MAGIC_OFFSET, CANMORE_LINUX_GEN_CONTROL_MAGIC_VALUE);

    uint64_t serialNum = discoverSerialNumber();
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_LOWER_FLASH_ID, serialNum & 0xFFFFFFFF);
    gen_control->addConstRegister(CANMORE_LINUX_GEN_CONTROL_UPPER_FLASH_ID, (serialNum >> 32) & 0xFFFFFFFF);

    gen_control->addCallbackRegister(CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                     bind_reg_cb(&CanmoreLinuxServer::restartDaemonCb));

    addRegisterPage(CANMORE_LINUX_GEN_CONTROL_PAGE_NUM, std::move(gen_control));

    addByteMappedPage(CANMORE_LINUX_VERSION_STRING_PAGE_NUM, REGISTER_PERM_READ_ONLY,
                      std::span<uint8_t>((uint8_t *) version_str, sizeof(version_str)));

    // Define TTY Control Page
    auto tty_control = Canmore::RegMappedRegisterPage::create();
    tty_control->addCallbackRegister(CANMORE_LINUX_TTY_CONTROL_ENABLE_OFFSET, REGISTER_PERM_READ_WRITE,
                                     bind_reg_cb(&CanmoreLinuxServer::enableTtyCb));
    tty_control->addMemoryRegister(CANMORE_LINUX_TTY_CONTROL_WINDOW_SIZE_OFFSET, REGISTER_PERM_READ_WRITE,
                                   &windowSzReg_);
    tty_control->addMemoryRegister(CANMORE_LINUX_TTY_CONTROL_USE_UPLOAD_DIR_OFFSET, REGISTER_PERM_READ_WRITE,
                                   &runCmdFromUploadDir_);
    addRegisterPage(CANMORE_LINUX_TTY_CONTROL_PAGE_NUM, std::move(tty_control));

    // Define terminal environment variable string
    termStrBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_TTY_TERMINAL_PAGE_NUM, REGISTER_PERM_READ_WRITE, termStrBuf_);

    // Define command string
    cmdBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_TTY_CMD_PAGE_NUM, REGISTER_PERM_READ_WRITE, cmdBuf_);

    // Define File Buffer page
    fileBuf_.resize(REG_MAPPED_PAGE_SIZE);
    addByteMappedPage(CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, REGISTER_PERM_READ_WRITE, fileBuf_);

    // Define File Upload Control page
    auto upload_control = Canmore::RegMappedRegisterPage::create();
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &filenameLengthReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET, REGISTER_PERM_READ_WRITE,
                                      &dataLengthReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET, REGISTER_PERM_READ_WRITE, &crc32Reg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_CLEAR_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &clearFileReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_READ_OFFSET_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                      &readOffsetReg_);
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET, REGISTER_PERM_READ_WRITE,
                                      &fileModeReg_);
    upload_control->addCallbackRegister(CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET, REGISTER_PERM_WRITE_ONLY,
                                        bind_reg_cb(&CanmoreLinuxServer::triggerFileOperation));
    upload_control->addMemoryRegister(CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET, REGISTER_PERM_READ_ONLY,
                                      &writeStatusReg_);
    addRegisterPage(CANMORE_LINUX_FILE_CONTROL_PAGE_NUM, std::move(upload_control));

    // initialize pwd for file operations
    file_pwd_ = getHome();
}

bool CanmoreLinuxServer::restartDaemonCb(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    if (*data_ptr == CANMORE_LINUX_GEN_CONTROL_RESTART_DAEMON_MAGIC) {
        shouldStop_ = true;
        return true;
    }
    else {
        return false;
    }
}

void CanmoreLinuxServer::getTtyInitialConfig(std::string &termEnv, uint16_t &initialRows, uint16_t &initialCols,
                                             std::string &cmd, std::string &workingDir) {
    // Decode window size register
    initialRows = windowSzReg_ >> 16;
    initialCols = (uint16_t) (windowSzReg_ & 0xFFFF);

    // Decode the terminal environment string
    const char *termEnvPtr = reinterpret_cast<char *>(termStrBuf_.data());
    size_t termEnvSize = strnlen(termEnvPtr, termStrBuf_.size());
    termEnv.assign(termEnvPtr, termEnvSize);

    // Decode the terminal command string
    const char *cmdPtr = reinterpret_cast<char *>(cmdBuf_.data());
    size_t cmdSize = strnlen(cmdPtr, cmdBuf_.size());
    cmd.assign(cmdPtr, cmdSize);

    // Set the working dir if we want to use the upload one
    if (runCmdFromUploadDir_) {
        workingDir.assign(file_pwd_);
    }
    else {
        workingDir.assign("");
    }
}

bool CanmoreLinuxServer::enableTtyCb(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    if (is_write) {
        if (*data_ptr == 1) {
            remoteTtyEnabled_ = true;
            return true;
        }
        else if (*data_ptr == 0) {
            remoteTtyEnabled_ = false;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        *data_ptr = remoteTtyEnabled_;
        return true;
    }
}

void CanmoreLinuxServer::doFileWrite() {
    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_BUSY;

    // check crc32
    uint32_t buf_crc = crc32_compute(fileBuf_.data(), dataLengthReg_);

    if (buf_crc != crc32Reg_) {
        writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC;
        return;
    }

    // clear file crc32 if necessary
    if (clearFileReg_) {
        currentFileCrc_ = 0xFFFFFFFF;
    }

    // read filename out of buffer page
    std::string filename = readFileNameFromBuf();

    try {
        // open file
        std::ofstream file;
        file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
        file.open(filename, (clearFileReg_ ? std::ofstream::trunc : std::ofstream::app));

        // do write
        size_t data_start = ((filenameLengthReg_ + 3) / 4) * 4;  // round up to next group of 4 bytes
        for (uint32_t i = data_start; i < dataLengthReg_; i++) {
            char chunk_int = (char) fileBuf_.at(i);
            file.write(&chunk_int, sizeof(chunk_int));
            currentFileCrc_ = crc32_update((uint8_t *) &chunk_int, 1, currentFileCrc_);
        }

        file.close();
    } catch (std::ofstream::failure &ex) {
        // write failure into filebuf and set error write status
        reportDeviceError(strerror(errno));
        return;  // if we dont return the file status will become success
    }

    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

void CanmoreLinuxServer::doFileRead() {
    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_BUSY;

    // update crc32
    if (clearFileReg_) {
        currentFileCrc_ = 0xFFFFFFFF;
    }

    // read file name out of buffer page
    std::string filename = readFileNameFromBuf();

    std::ifstream file;
    file.open(filename);

    if (!file.good()) {
        reportDeviceError(strerror(errno));
        return;  // or else status becomes SUCCESS
    }

    // go to desired offset and read there
    file.seekg(readOffsetReg_);

    // do read
    file.read((char *) readBuf_, sizeof(readBuf_));
    size_t num_read = file.gcount();

    file.close();
    fileBuf_.assign(readBuf_, readBuf_ + sizeof(readBuf_));

    // set data length register
    dataLengthReg_ = num_read;

    // set crc32 register
    crc32Reg_ = crc32_compute(fileBuf_.data(), num_read);

    // update file crc32
    currentFileCrc_ = crc32_update(fileBuf_.data(), num_read, currentFileCrc_);

    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

void CanmoreLinuxServer::doSetFileMode() {
    mode_t new_mode = (mode_t) fileModeReg_;
    std::string filename = readFileNameFromBuf();
    if (chmod(filename.c_str(), new_mode) < 0) {
        reportDeviceError(strerror(errno));
        return;  // return or else file status becomes success
    }

    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

void CanmoreLinuxServer::doGetFileMode() {
    struct stat fileStat;
    std::string filename = readFileNameFromBuf();
    if (stat(filename.c_str(), &fileStat) < 0) {
        reportDeviceError(strerror(errno));
        return;  // return or else file status becomes SUCCESS
    }

    fileModeReg_ = (uint32_t) fileStat.st_mode;
    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

void CanmoreLinuxServer::doGetFileLen() {
    std::string filename = readFileNameFromBuf();
    try {
        dataLengthReg_ = (uint32_t) std::filesystem::file_size(filename);
        writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
    } catch (std::filesystem::filesystem_error &ex) {
        reportDeviceError(strerror(ex.code().value()));
    }
}

void CanmoreLinuxServer::doCheckCrc() {
    writeStatusReg_ =
        (crc32Reg_ == currentFileCrc_ ? CANMORE_LINUX_FILE_STATUS_SUCCESS : CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC);
}

void CanmoreLinuxServer::doCd() {
    std::string filename = readFileNameFromBuf();
    std::string dir;
    if (!readFiletoString(filename, dir)) {
        reportDeviceError(strerror(errno));
        return;
    }

    if (dir == "") {
        file_pwd_ = getHome();
    }

    if (!isAbsolutePath(dir)) {
        dir = joinPaths(file_pwd_, dir);
    }

    bool exists = std::filesystem::exists(dir), is_directory = std::filesystem::is_directory(dir);
    if (exists && is_directory) {
        file_pwd_ = std::filesystem::canonical(dir);
        writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
        return;
    }

    // if we got here, the directory does not exist
    if (!exists)
        reportDeviceError("Does not exist");
    else if (!is_directory)
        reportDeviceError("Not a direcory");
}

void CanmoreLinuxServer::doLs() {
    // read name of tmp file out of buffer. will have the name of the directory to list, and we will also store results
    // in that file.
    std::string filename = readFileNameFromBuf();
    std::string dirname;
    if (!readFiletoString(filename, dirname)) {
        reportDeviceError(strerror(errno));
        return;
    }

    if (!isAbsolutePath(dirname)) {
        dirname = joinPaths(file_pwd_, dirname);
    }

    if (!std::filesystem::exists(dirname)) {
        reportDeviceError("No such directory");
        return;
    }

    // now list the directory into a string
    std::string report = "";
    auto dir_it = std::filesystem::directory_iterator(dirname);
    for (const auto &entry : dir_it) {
        report += entry.path().filename().string() + "\n";
    }

    // write report to file and transfer it
    if (!writeStringToFile(filename, report)) {
        reportDeviceError(strerror(errno));
        return;
    }

    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

void CanmoreLinuxServer::doPwd() {
    std::string filename = readFileNameFromBuf();
    if (!writeStringToFile(filename, file_pwd_)) {
        reportDeviceError(strerror(errno));
        return;
    }

    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_SUCCESS;
}

bool CanmoreLinuxServer::triggerFileOperation(uint16_t addr, bool is_write, uint32_t *data_ptr) {
    bool want_operation = *data_ptr != CANMORE_LINUX_FILE_OPERATION_NOP;
    if (current_operation_ == CANMORE_LINUX_FILE_OPERATION_NOP && want_operation) {
        switch (*data_ptr) {
        case CANMORE_LINUX_FILE_OPERATION_WRITE:
            doFileWrite();
            break;
        case CANMORE_LINUX_FILE_OPERATION_READ:
            doFileRead();
            break;
        case CANMORE_LINUX_FILE_OPERATION_SET_MODE:
            doSetFileMode();
            break;
        case CANMORE_LINUX_FILE_OPERATION_GET_MODE:
            doGetFileMode();
            break;
        case CANMORE_LINUX_FILE_OPERATION_GET_FILE_LEN:
            doGetFileLen();
            break;
        case CANMORE_LINUX_FILE_OPERATION_CHECK_CRC:
            doCheckCrc();
            break;
        case CANMORE_LINUX_FILE_OPERATION_CD:
            doCd();
            break;
        case CANMORE_LINUX_FILE_OPERATION_LS:
            doLs();
            break;
        case CANMORE_LINUX_FILE_OPERATION_PWD:
            doPwd();
            break;
        default:
            reportDeviceError("Unknown operation");
        }
    }

    if (!want_operation) {
        writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_READY;
    }

    current_operation_ = want_operation;
    return true;
}

void CanmoreLinuxServer::reportDeviceError(const char *error) {
    size_t error_len = strlen(error);
    if (error_len > REG_MAPPED_PAGE_SIZE) {
        error_len = REG_MAPPED_PAGE_SIZE;
    }

    for (size_t i = 0; i < error_len; i++) {
        fileBuf_[i] = error[i];
    }
    dataLengthReg_ = error_len;
    writeStatusReg_ = CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR;
}

std::string CanmoreLinuxServer::readFileNameFromBuf() {
    char filename[filenameLengthReg_ + 1] = { 0 };
    for (uint32_t i = 0; i < filenameLengthReg_; i++) {
        filename[i] = (char) fileBuf_.at(i);
    }

    std::string ret = filename;
    if (!isAbsolutePath(ret)) {
        ret = joinPaths(file_pwd_, ret);
    }

    return ret;
}

void CanmoreLinuxServer::forceTTYdisconnect() noexcept {
    // Sends the tty disconnect (should be sent on startup) to kick any canmmore cli instances out of the remote sh mode
    canid_t can_id = CAN_EFF_FLAG | CANMORE_REMOTE_TTY_CALC_ID_C2A(clientId, CANMORE_REMOTE_TTY_SUBCH_CONTROL,
                                                                   CANMORE_REMOTE_TTY_CMD_DISCONNECT_ID);

    canmore_remote_tty_cmd_disconnect_t pkt = { .pkt = { .is_err = true } };
    transmitFrameNoexcept(can_id, pkt.data, sizeof(pkt));
}

bool CanmoreLinuxServer::readFiletoString(const std::string &filename, std::string &contents) {
    std::ifstream in;
    in.open(filename);
    if (!in) {
        return false;
    }

    size_t size = std::filesystem::file_size(filename);
    contents.resize(size);
    in.read(contents.data(), size);
    return true;
}

bool CanmoreLinuxServer::writeStringToFile(const std::string &filename, const std::string &contents) {
    std::ofstream out;
    out.open(filename);
    if (out) {
        out << contents;
        return true;
    }

    return false;
}

std::string CanmoreLinuxServer::joinPaths(const std::string &path1, const std::string &path2) {
    std::string p1 = path1, p2 = path2;

    if (p1.size() != 0 && p1.at(p1.size() - 1) == '/') {
        p1 = p1.substr(0, p1.length() - 1);
    }

    if (p2.size() != 0 && p2.at(0) == '/') {
        p2 = p2.substr(1);
    }

    return p1 + "/" + p2;
}

std::string CanmoreLinuxServer::getHome() {
    const char *home = getenv("HOME");
    if (home) {
        // HOME was gotten successfully
        return home;
    }
    else {
        // HOME not gotten successfully, fall back to cwd
        return std::filesystem::current_path();
    }
}
