cmake_minimum_required(VERSION 3.12)
project(takephoto)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} include)
# add_executable(runCameras libcoral_bindings/main.cpp tools/mySerial.cpp)
# add_executable(cameraDetect src/main.cpp tools/mySerial.cpp)
add_executable(intrCalibration calibration_scripts/cameraCalibration.cpp tools/mySerial.cpp)
add_executable(stereoCalibration calibration_scripts/stereoCalibration.cpp tools/mySerial.cpp)

# target_link_directories(runCameras PUBLIC libcoral_bindings/out/)
# target_link_libraries(runCameras -llccv ${OpenCV_LIBS} -lcoral_yolo)

# target_link_directories(cameraDetect PUBLIC libcoral_bindings/out/)
# target_link_libraries(cameraDetect -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_libraries(intrCalibration -llccv ${OpenCV_LIBS})
target_link_libraries(stereoCalibration -llccv ${OpenCV_LIBS})