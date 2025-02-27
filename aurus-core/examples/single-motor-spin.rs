#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use stm32::{init_peripherals, MotorDriver};
use {defmt_rtt as _, panic_probe as _};

const ON_TIME_MS: u64 = 2000; // How long each motor stays on
const OFF_TIME_MS: u64 = ON_TIME_MS; // How long each motor stays off

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize peripherals
    let p = init_peripherals();

    // Create motor drivers
    let mut motor = MotorDriver::new(p.PA0, p.PA1);

    loop {
        info!("Motor on");
        motor.motor_on();
        Timer::after_millis(ON_TIME_MS).await;

        info!("Motor off");
        motor.motor_off();
        Timer::after_millis(OFF_TIME_MS).await;
    }
}