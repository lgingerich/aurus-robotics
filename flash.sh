#!/bin/bash

set -e

echo "Building release version..."
cargo build --release

echo "Converting to binary format..."
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/embedded-rust-learning firmware.bin

echo "Flashing firmware..."
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin

echo "Flash complete!"