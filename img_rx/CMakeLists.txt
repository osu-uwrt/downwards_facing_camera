cmake_minimum_required(VERSION 3.1)
project(img-gateway)


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../titan_canmore titan_canmore)


add_executable(img-gateway main.cpp CanmoreImageReceiver.cpp CameraSocketListener.cpp)
target_link_libraries(img-gateway canmore_cpp)
