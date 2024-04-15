#!/bin/bash

set -e

cd "$(dirname "$0")"

LIBRARY_GEN_SCRIPT="microros_static_library/library_generation/library_generation.sh"
if ! [ -f "$LIBRARY_GEN_SCRIPT" ]; then
    echo "Could not find $LIBRARY_GEN_SCRIPT"
    echo "Ensure build.sh is in the correct directory"
    exit 1
fi

if ! [ -x "$LIBRARY_GEN_SCRIPT" ]; then
    echo "$LIBRARY_GEN_SCRIPT is not executable!"
    echo "Chmod the file and try again"
    exit 1
fi

if ! command -v docker > /dev/null; then
    if [ "$(uname -m)" == "aarch64" ]; then
        echo "You do not have docker installed"
        echo "It is recommended that you run this on a DIFFERNET MACHINE with docker, then transfer the resulting archive to this machine"
        echo "Then, run the ./extract_crosscompile.sh script to extract the libmicroros build"
        exit 1
    else
        echo "Docker is not installed. Please install docker to cross compile"
        exit 1
    fi
fi

docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble
