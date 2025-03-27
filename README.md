# Aurus Robotics Core

This repository contains the core firmware for Aurus, a robotics development framework written in Rust.

> **Note:** This project is still heavily in development. Features and APIs are subject to change, and the documentation may not always be up-to-date.

## Flashing Methods

### Method 1: Using J-Link Debugger (Recommended)

1. Connect your J-Link to the STM32 board:
   - GND → STM32 GND
   - SWDIO → STM32 PA13
   - SWCLK → STM32 PA14
   - VTref → STM32 3V3

2. Power the STM32 board via USB-C

3. Run an example:
```bash
cd aurus-core
cargo run --example single_motor_spin_gpio
```

The program will be automatically flashed and started. Debug output will be shown in the terminal.

### Method 2: Using DFU (Direct Firmware Update)

If you don't have a debugger, you can use the built-in DFU mode:

1. Check if device is detected:
```bash
dfu-util -l
```

2. Enter DFU mode:
   - Press and hold BOOT0
   - While holding BOOT0, press NRST for 1 second
   - Release NRST, then release BOOT0

3. Flash the board:
```bash
./flash.sh
```

## Development

### Running Examples
```bash
# List all available examples
cargo run --example

# Run a specific example
cargo run --example single_motor_spin_gpio

# Run with detailed logging
DEFMT_LOG=trace cargo run --example single_motor_spin_gpio
```

### Debugging
When using the J-Link debugger, you'll see:
- Real-time program execution logs via RTT (Real-Time Transfer)
- Error messages and panic information
- Program state and peripheral initialization status