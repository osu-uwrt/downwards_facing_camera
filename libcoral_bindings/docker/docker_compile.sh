#!/bin/bash

if [ -z "$1" ]; then
    echo "Missing target name argument"
    echo "Usage: ./docker_compile.sh [target] [optional new ownership]"
    exit 1
fi

set -e
# Add our "custom package" from the /workspace dir
cd /libcoral
ln -s /workspace coral/custom_package

# Build the custom package target (whatever its set to in BUILD)
bazel build --compilation_mode=opt --cpu=aarch64 --copt=-ffp-contract=off "//coral/custom_package:$1"

# Copy the output artifact to the workspace directory
mkdir -p "/workspace/out"
cp "bazel-out/aarch64-opt/bin/coral/custom_package/$1" "/workspace/out/lib$1.so"
chmod +w "/workspace/out/lib$1.so"
chmod -x "/workspace/out/lib$1.so"

# Change ownership of the output file
if ! [ -z "$2" ]; then
    chown -R "$2" "/workspace/out"
fi
