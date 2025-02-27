#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use stm32::{init_peripherals, MotorDriver};
use {defmt_rtt as _, panic_probe as _};

const ON_TIME_MS: u64 = 2000; // How long each motor stays on
const OFF_TIME_MS: u64 = ON_TIME_MS; // How long each motor stays off
const OVERLAP_TIME_MS: u64 = ON_TIME_MS / 2; // Half the on time as overlap

#[embassy_executor::task]
async fn motor1_task(mut motor1: MotorDriver) {
    loop {
        info!("Motor 1 on");
        motor1.motor_on();
        Timer::after_millis(ON_TIME_MS).await;
        
        info!("Motor 1 off");
        motor1.motor_off();
        Timer::after_millis(OFF_TIME_MS).await;
    }
}

#[embassy_executor::task]
async fn motor2_task(mut motor2: MotorDriver) {
    // Initial delay to offset motor2's start time
    Timer::after_millis(ON_TIME_MS - OVERLAP_TIME_MS).await;
    loop {
        info!("Motor 2 on");
        motor2.motor_on();
        Timer::after_millis(ON_TIME_MS).await;
        
        info!("Motor 2 off");
        motor2.motor_off();
        Timer::after_millis(OFF_TIME_MS).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize peripherals
    let p = init_peripherals();

    // Create motor drivers
    let motor1 = MotorDriver::new(p.PA0, p.PA1);
    let motor2 = MotorDriver::new(p.PA6, p.PA7);

    // Spawn tasks for each motor individually
    spawner.spawn(motor1_task(motor1)).unwrap();
    spawner.spawn(motor2_task(motor2)).unwrap();

    loop {
        Timer::after_millis(10000).await;
    }
}

