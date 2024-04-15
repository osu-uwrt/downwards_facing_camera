cmake_minimum_required(VERSION 3.1)
project(img-encoder)
SET("OpenCV_DIR" "/usr/share/OpenCV")
find_package( OpenCV REQUIRED )
find_package( JPEG REQUIRED)

set(NO_AMENT 1)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../titan_canmore titan_canmore)

add_executable(img-tx main.cpp ../../tools/CanmoreImageTransmitter.cpp)
target_include_directories(img-tx PRIVATE ../../include ${JPEG_INCLUDE_DIR})
target_link_libraries(img-tx ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
