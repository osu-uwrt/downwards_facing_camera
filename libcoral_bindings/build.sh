#!/bin/sh

TARGET_NAME=coral_yolo
DOCKER_IMAGE_NAME=libcoral_build_env

set -e

if ! command -v docker > /dev/null; then
    echo "Docker is not installed. Please install docker to use this script"
    exit 1
fi

if [ -z "$(docker images -q "$DOCKER_IMAGE_NAME" 2> /dev/null)" ]; then
    echo "Building docker image..."
    docker build docker/ -t "$DOCKER_IMAGE_NAME"
    exit 1
fi

docker run --rm -it -v "$(pwd):/workspace" "$DOCKER_IMAGE_NAME" /bin/bash \
    /workspace/docker/docker_compile.sh "$TARGET_NAME" "$(id -u):$(id -g)"

scp out/libcoral_yolo.so pi@raspberrypi:/home/pi/camSW
