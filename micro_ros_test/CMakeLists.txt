cmake_minimum_required(VERSION 3.12)

project(microros_test)

set(NO_AMENT 1)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../titan_canmore titan_canmore)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../micro_ros_raspi micro_ros_raspi)

add_executable(microros_test main.cpp DFCMicroROSClient.cpp)

target_link_libraries(microros_test PUBLIC micro_ros)
