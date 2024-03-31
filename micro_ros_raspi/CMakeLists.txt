cmake_minimum_required(VERSION 3.12)

# Define sources
add_library(micro_ros INTERFACE)
target_sources(micro_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/micro_ros_bindings.cpp
)

target_link_libraries(micro_ros INTERFACE
    canmore_cpp
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/libmicroros.a
)

target_include_directories(micro_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)
