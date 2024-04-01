cmake_minimum_required(VERSION 3.1)
project(img-encoder)
SET("OpenCV_DIR" "/usr/share/OpenCV")
find_package( OpenCV REQUIRED )
find_package( JPEG REQUIRED)


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../titan_canmore titan_canmore)


add_executable(img-encoder main.cpp CanmoreImageTransmitter.cpp)
target_include_directories(img-encoder PRIVATE ${JPEG_INCLUDE_DIR})
target_link_libraries(img-encoder ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
