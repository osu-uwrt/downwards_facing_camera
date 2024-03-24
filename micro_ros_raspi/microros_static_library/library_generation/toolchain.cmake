
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_CROSSCOMPILING 1)
# set(CMAKE_SYSTEM_PROCESSOR cortex-a76)
# set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc CACHE FILEPATH "C compiler")
# set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++ CACHE FILEPATH "C++ compiler")
# set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
# set(CMAKE_SYSROOT /usr/aarch64-linux-gnu)

# set(FLAGS "-O3 -march=armv8.2-a -mcpu=cortex-a76 -ffunction-sections -fdata-sections -fno-exceptions -nostdlib -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)
set(FLAGS "-O3 -ffunction-sections -fdata-sections -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS}" CACHE STRING "" FORCE)
