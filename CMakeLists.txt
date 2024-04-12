cmake_minimum_required(VERSION 3.12)
project(takephoto)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(JPEG REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

set(NO_AMENT 1)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/titan_canmore titan_canmore)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} include)
# add_executable(runCameras libcoral_bindings/main.cpp tools/mySerial.cpp)
add_executable(cameraDetect src/main.cpp tools/mySerial.cpp)
add_executable(intrCalibration calibration_scripts/cameraCalibration.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(stereoCalibration calibration_scripts/stereoCalibration.cpp tools/mySerial.cpp)

add_custom_target(calibration DEPENDS intrCalibration stereoCalibration)

set_target_properties(intrCalibration PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(stereoCalibration PROPERTIES EXCLUDE_FROM_ALL TRUE)

# target_link_directories(runCameras PUBLIC libcoral_bindings/out/)
# target_link_libraries(runCameras -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_directories(cameraDetect PUBLIC libcoral_bindings/out/)
target_link_libraries(cameraDetect -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_directories(cameraDetect PRIVATE ${JPEG_INCLUDE_DIR})
target_link_libraries(intrCalibration -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
target_link_libraries(stereoCalibration -llccv ${OpenCV_LIBS})
