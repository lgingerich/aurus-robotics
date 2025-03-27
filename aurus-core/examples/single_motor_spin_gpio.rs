#![no_std]
#![no_main]

use aurus_core::motor::gpio::GpioMotorDriver;
use aurus_core::motor::traits::MotorDriver;

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const ON_TIME_MS: u64 = 2000; // How long each motor stays on
const OFF_TIME_MS: u64 = ON_TIME_MS; // How long each motor stays off

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Print program start
    info!("Starting motor control program...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Peripherals initialized");

    // Create motor driver config
    let config = (p.PA0.into(), p.PA1.into());

    // Create motor drivers
    let mut motor: GpioMotorDriver = MotorDriver::new(config);
    info!("Motor driver initialized");

    info!("Entering main control loop");
    loop {
        info!("Setting direction: forward");
        motor.set_direction(true);
        Timer::after_millis(ON_TIME_MS).await;

        info!("Starting motor");
        motor.start();
        Timer::after_millis(ON_TIME_MS).await;

        info!("Setting direction: reverse");
        motor.set_direction(false);
        Timer::after_millis(ON_TIME_MS).await;

        info!("Stopping motor");
        motor.stop();
        Timer::after_millis(OFF_TIME_MS).await;
    }
}
