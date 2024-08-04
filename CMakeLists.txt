cmake_minimum_required(VERSION 3.12)
project(takephoto)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(JPEG REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

set(NO_AMENT 1)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/titan_canmore titan_canmore)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_raspi micro_ros_raspi)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} include micro_ros_raspi/include micro_ros_raspi/libmicroros/include)
add_executable(runCameras src/runCameras.cpp tools/mySerial.cpp)
add_executable(cameraDetect src/main.cpp tools/mySerial.cpp)
add_executable(arucoCal calibration_scripts/arucoCalibration.cpp calibration_scripts/pointGenerator.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(arucoSterCal calibration_scripts/arucoStereoCalibration.cpp calibration_scripts/pointGenerator.cpp tools/mySerial.cpp tools/CanmoreImageTransmitter.cpp)
add_executable(takeSterImage tools/takeStereoIm.cpp tools/mySerial.cpp)
add_executable(runAgents src/runAgents.cpp tools/mySerial.cpp)
add_executable(yoloInterpreter src/yoloInterpreter.cpp)

add_library(agentLib src/agents/cameraAgent.cpp src/agents/orientationAgent.cpp src/agents/yoloAgent.cpp tools/mySerial.cpp tools/DFCMicroROSClient.cpp)
target_link_directories(agentLib PUBLIC libcoral_bindings/out/)
target_link_libraries(agentLib -llccv ${OpenCV_LIBS} -lcoral_yolo canmore_cpp)

add_custom_target(calibration DEPENDS arucoCal arucoSterCal)
add_custom_target(tools DEPENDS takeSterImage)
add_custom_target(agents DEPENDS runAgents)

set_target_properties(arucoCal PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(arucoSterCal PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(takeSterImage PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(runAgents PROPERTIES EXCLUDE_FROM_ALL TRUE)
set_target_properties(agentLib PROPERTIES EXCLUDE_FROM_ALL TRUE)

# target_link_directories(runCameras PUBLIC libcoral_bindings/out/)
target_link_libraries(runCameras -llccv ${OpenCV_LIBS})

target_link_directories(cameraDetect PUBLIC libcoral_bindings/out/)
target_link_libraries(cameraDetect -llccv ${OpenCV_LIBS} -lcoral_yolo)

target_link_directories(cameraDetect PRIVATE ${JPEG_INCLUDE_DIR})
target_link_libraries(arucoCal -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)
target_link_libraries(arucoSterCal -llccv ${OpenCV_LIBS} ${JPEG_LIBRARIES} canmore_cpp)

target_link_libraries(takeSterImage -llccv ${OpenCV_LIBS})

target_link_libraries(runAgents agentLib micro_ros)

target_link_directories(yoloInterpreter PUBLIC libcoral_bindings/out/)
target_link_libraries(yoloInterpreter -lcoral_yolo)
