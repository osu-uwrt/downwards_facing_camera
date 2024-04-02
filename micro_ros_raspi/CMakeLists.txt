cmake_minimum_required(VERSION 3.12)

set(MICRO_ROS_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/libmicroros/include")
set(MICRO_ROS_LIB_FILE "${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a")
if((NOT EXISTS "${MICRO_ROS_INCLUDE_DIR}") OR (NOT EXISTS "${MICRO_ROS_LIB_FILE}"))
    message(FATAL_ERROR "Could not locate prebuilt libmicroros install!\n"
            "Please clone this repository on a machine with docker and run the ./build.sh command. Then transfer the "
            "resulting 'libmicroros_aarch64.tgz' to this machine and run the ./extract_crosscompile.sh command.")
endif()

# Define sources
add_library(micro_ros INTERFACE)
target_sources(micro_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/micro_ros_bindings.cpp
)

target_link_libraries(micro_ros INTERFACE
    canmore_cpp
    ${MICRO_ROS_LIB_FILE}
)

target_include_directories(micro_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_INCLUDE_DIR}
)
