#![no_std]
#![no_main]

use aurus_core::{
    motor::gpio::DigitalOutput,
    motor::motor::Motor,
};
use embassy_stm32::{
    gpio::{Level, Output, OutputType, Pull, Speed},
    peripherals::{PA0, PA1},
};

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const ON_TIME_MS: u64 = 2000; // How long each motor stays on
const OFF_TIME_MS: u64 = ON_TIME_MS; // How long each motor stays off

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Print program start
    info!("Starting capability-driven GPIO motor control program...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Peripherals initialized");

    // Configure GPIO pins
    let direction_pin = Output::new(
        p.PA0,
        Level::Low,
        Speed::Low,
    );
    
    let enable_pin = Output::new(
        p.PA1,
        Level::Low,
        Speed::Low,
    );

    // Create motor with our GPIO capabilities
    let mut motor = Motor::new(direction_pin, (), Some(enable_pin));
    info!("Motor initialized with GPIO capabilities");

    info!("Entering main control loop");
    loop {
        // Forward direction
        info!("Setting direction: forward");
        motor.set_direction(true).unwrap();
        motor.start().unwrap();
        Timer::after_millis(ON_TIME_MS).await;

        // Stop
        info!("Stopping motor");
        motor.stop().unwrap();
        Timer::after_millis(OFF_TIME_MS).await;

        // Reverse direction
        info!("Setting direction: reverse");
        motor.set_direction(false).unwrap();
        motor.start().unwrap();
        Timer::after_millis(ON_TIME_MS).await;

        // Stop
        info!("Stopping motor");
        motor.stop().unwrap();
        Timer::after_millis(OFF_TIME_MS).await;
    }
} 