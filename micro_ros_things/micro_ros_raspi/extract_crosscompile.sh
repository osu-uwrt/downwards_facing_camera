#!/bin/bash

set -e
cd "$(dirname "$0")"

ARCHIVE_FILE="libmicroros_aarch64.tgz"

if [ "$(uname -m)" != "aarch64" ]; then
    echo "Unable to extract cross compiled libmicroros build on non-aarch64 machine!"
    exit 1
fi

if ! [ -f "$ARCHIVE_FILE" ]; then
    echo "Did not find '$ARCHIVE_FILE' in this directory!"
    echo "Run the ./build.sh command on a machine with docker, then transfer that file to this directory"
    exit 1
fi

tar xzf "$ARCHIVE_FILE"
