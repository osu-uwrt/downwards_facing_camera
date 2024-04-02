#!/bin/bash

set -e

apt update
apt -y install rsync

######## Configure Cross Compile ########

# Note from author: If you want to compile it for your local machine,
# you can comment out this out and it'll happily make an x86 build
if [ "$(uname -m)" != "aarch64" ]; then
    echo "Detected running on non-aarch64 machine: performing cross compile"
    export UROS_RPI_CROSS_COMPILE=1
    export DEST_DIR=/output_staging
    mkdir "$DEST_DIR"
else
    export DEST_DIR=/project
fi

######## Init ########

cd /uros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

######## Adding extra packages ########
pushd firmware/mcu_ws > /dev/null

    # Workaround: Copy just tf2_msgs
    git clone -b humble https://github.com/ros2/geometry2
    cp -R geometry2/tf2_msgs ros2/tf2_msgs
    rm -rf geometry2

    # Another workaround for vision msgs
    git clone -b ros2 https://github.com/ros-perception/vision_msgs
    cp -R vision_msgs/vision_msgs ros2/vision_msgs
    rm -rf vision_msgs

    # Import user defined packages
    mkdir extra_packages
    pushd extra_packages > /dev/null
        cp -R /project/microros_static_library/library_generation/extra_packages/* .
        vcs import --input extra_packages.repos
    popd > /dev/null

    # Workaround: The software team enjoys pain.
    git clone https://github.com/osu-uwrt/riptide_core
    cp -R riptide_core/riptide_msgs ros2/riptide_msgs
    rm -rf riptide_core

popd > /dev/null

######## Clean old builds ########
rm -rf "$DEST_DIR/libmicroros/include"
rm -f "$DEST_DIR/libmicroros/libmicroros.a"
rm -f "$DEST_DIR/built_packages"
rm -f "$DEST_DIR/available_ros2_types"

######## Build for Raspberry Pi 5  ########
rm -rf firmware/build

if [ $UROS_RPI_CROSS_COMPILE ]; then
    apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu libgcc-11-dev-arm64-cross
fi

ros2 run micro_ros_setup build_firmware.sh /project/microros_static_library/library_generation/toolchain.cmake /project/microros_static_library/library_generation/colcon.meta

find firmware/build/include/ -name "*.c"  -delete
mkdir -p "$DEST_DIR/libmicroros/include"
cp -R firmware/build/include/* "$DEST_DIR/libmicroros/include"

cp firmware/build/libmicroros.a "$DEST_DIR/libmicroros/libmicroros.a"

######## Fix include paths  ########
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$DEST_DIR/libmicroros/include/${var}/${var}" ]; then
        rsync -r $DEST_DIR/libmicroros/include/${var}/${var}/* "$DEST_DIR/libmicroros/include/${var}"
        rm -rf "$DEST_DIR/libmicroros/include/${var}/${var}"
    fi
done

######## Generate extra files ########
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > "$DEST_DIR/available_ros2_types"
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> "$DEST_DIR/available_ros2_types"

cd firmware
echo "" > "$DEST_DIR/built_packages"
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> "$DEST_DIR/built_packages"; popd > /dev/null; done;
# sort it so that the result order is reproducible
sort -o "$DEST_DIR/built_packages" "$DEST_DIR/built_packages"

######## Fix permissions ########
if [ "$DEST_DIR" == "/project" ]; then
    chmod -R 777 /project/libmicroros
    chmod -R -x+X /project/libmicroros
else
    # We aren't running in /project, archive it into the project directory
    tar -C "$DEST_DIR" -c -z -f /project/libmicroros_aarch64.tgz .
fi

echo
echo ========================================
echo Successfully Compiled MicroROS
echo ========================================
