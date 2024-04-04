#!/bin/sh

set -e

SCRIPT_DIR="$(realpath "$(dirname "$0")")"

# If path has not been explicitly set, just rely on the path to find it
if [ -z "$EEPMAKE_PATH" ]; then
    EEPMAKE_PATH=eepmake
elif [ -d "$EEPMAKE_PATH" ]; then
    # Path is a directory, assume eepmake is in that folder
    EEPMAKE_PATH="$EEPMAKE_PATH/eepmake"
fi

# Create temporary directory (and delete it when we exit)
tmpdir=$(mktemp -d eeprom-compile.XXXXXXXXXX)
trap "rm -rf $tmpdir" EXIT

# Compile the device tree source
dtc -I dts "$SCRIPT_DIR/ext-cam-hat-overlay.dtso" -O dtb -o "$tmpdir/ext-cam-hat.dtbo"

# Compile v1 hat info
$EEPMAKE_PATH -v1 "$SCRIPT_DIR/ext-cam-hat-v1-settings.txt" "$SCRIPT_DIR/ext-cam-hat.eep" "$tmpdir/ext-cam-hat.dtbo"
