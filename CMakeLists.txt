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
add_executable(runCameras libcoral_bindings/main.cpp tools/mySerial.cpp)
add_executable(cameraDetect src/main.cpp tools/mySerial.cpp)
# add_executable(intrCalibration calibration_scripts/cameraCalibration.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
# add_executable(stereoCalibration calibration_scripts/stereoCalibration.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(arucoCal calibration_scripts/arucoCalibration.cpp calibration_scripts/pointGenerator.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(arucoSterCal calibration_scripts/arucoStereoCalibration.cpp calibration_scripts/pointGenerator.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(takeSterImage tools/takeStereoIm.cpp tools/mySerial.cpp)

# add_custom_target(calibration DEPENDS intrCalibration stereoCalibration arucoCal arucoSterCal)
add_custom_target(calibration DEPENDS arucoCal arucoSterCal)
add_custom_target(tools DEPENDS takeSterImage)

# set_target_properties(intrCalibration PROPERTIES EXCLUDE_FROM_ALL TRUE)
# set_target_properties(stereoCalibration PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(arucoCal PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(arucoSterCal PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(takeSterImage PROPERTIES EXCLUDE_FROM_ALL TRUE)

target_link_directories(runCameras PUBLIC libcoral_bindings/out/)
target_link_libraries(runCameras -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_directories(cameraDetect PUBLIC libcoral_bindings/out/)
target_link_libraries(cameraDetect -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_directories(cameraDetect PRIVATE ${JPEG_INCLUDE_DIR})
# target_link_libraries(intrCalibration -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
# target_link_libraries(stereoCalibration -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
target_link_libraries(arucoCal -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
target_link_libraries(arucoSterCal -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)

target_link_libraries(takeSterImage -llccv ${OpenCV_LIBS})