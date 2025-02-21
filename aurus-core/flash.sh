#!/bin/bash

set -e

echo "Building release version..."
cargo build --release

BINARY_PATH="target/thumbv7em-none-eabihf/release/aurus-core"

# Check if binary exists and has size
echo "Checking binary..."
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Binary not found at $BINARY_PATH"
    exit 1
fi

echo "Binary size: $(ls -lh $BINARY_PATH)"

echo "Converting to binary format..."
# Added more specific options to ensure proper section copying
arm-none-eabi-objcopy -v -O binary \
    --strip-all \
    --strip-debug \
    --strip-unneeded \
    "$BINARY_PATH" firmware.bin

# Verify file size
if [ ! -s firmware.bin ]; then
    echo "Error: firmware.bin is empty!"
    echo "objcopy command failed. Exit code: $?"
    echo "Input binary type:"
    file "$BINARY_PATH"
    # Add section information
    echo "ELF sections:"
    arm-none-eabi-objdump -h "$BINARY_PATH"
    exit 1
fi

echo "firmware.bin size: $(ls -lh firmware.bin)"

echo "Flashing firmware..."
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin

echo "Flash complete!"