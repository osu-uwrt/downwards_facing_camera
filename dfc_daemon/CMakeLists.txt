cmake_minimum_required(VERSION 3.12)

# Find the titan_firmware dir
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)

project(dfc_daemon)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add Canmore CPP Core Libraries
add_subdirectory(${REPO_DIR}/titan_canmore titan_canmore EXCLUDE_FROM_ALL)

# Find threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Check if this system is running systemd
find_package(PkgConfig REQUIRED)
include(cmake/systemdservice.cmake)

# If so, we need to have the libsystemd library to notify of service events
if (SYSTEMD_FOUND)
    pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)
    add_compile_definitions(HAS_SYSTEMD=1)
endif()

# Create upload tool
add_executable(dfc_daemon
    src/main.cpp
    src/HeartbeatTransmitter.cpp
    src/CanmoreLinuxServer.cpp
    src/PTYMaster.cpp
    src/RemoteTTYServer.cpp
    src/SerialLookup.cpp
    src/SystemdNotifier.cpp
)

target_include_directories(dfc_daemon PRIVATE include/)
target_compile_options(dfc_daemon PRIVATE -Wno-unused-parameter -Wall -Wextra)

target_link_libraries(dfc_daemon
    canmore_cpp
    Threads::Threads
)

if (SYSTEMD_FOUND)
    target_link_libraries(dfc_daemon PkgConfig::SYSTEMD)
    target_compile_definitions(dfc_daemon PRIVATE HAS_SYSTEMD=1)
endif()

# Create install definition
install(TARGETS dfc_daemon RUNTIME DESTINATION bin)

if (SYSTEMD_FOUND)
    set(DFC_DAEMON_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin/dfc_daemon)

    # Determine what user/group the service should run as
    execute_process(COMMAND id -u -n OUTPUT_VARIABLE DFC_DAEMON_RUN_USER OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND id -g -n OUTPUT_VARIABLE DFC_DAEMON_RUN_GROUP OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "=== Configuring daemon to run as user: '${DFC_DAEMON_RUN_USER}'; group: '${DFC_DAEMON_RUN_GROUP}'")

    configure_file(systemd/dfc_daemon@.service.in ${CMAKE_CURRENT_BINARY_DIR}/dfc_daemon@.service @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dfc_daemon@.service DESTINATION ${SYSTEMD_SERVICES_INSTALL_DIR})
endif()
