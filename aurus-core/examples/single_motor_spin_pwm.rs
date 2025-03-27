#![no_std]
#![no_main]

use aurus_core::motor::pwm::PwmMotorDriver;
use aurus_core::motor::traits::{MotorDriver, SpeedControl};

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const PWM_FREQ: u32 = 4000; // PWM frequency in Hz
const STEP_TIME_MS: u64 = 1000; // Time between speed changes
const MAX_SPEED: u8 = 100; // Maximum speed (100%)
const MIN_SPEED: u8 = 0; // Minimum speed (0%)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Print program start
    info!("Starting PWM motor control program...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Peripherals initialized");

    // Create motor driver config
    let config = (
        p.TIM1,
        p.PA8,
        p.PA1.into(), // TODO: Should not have to call into() on what is supposed to be user facing code
        PWM_FREQ,
    );

    // Create motor driver
    let mut motor: PwmMotorDriver = MotorDriver::new(config);
    info!("Motor driver initialized");

    info!("Entering main control loop");
    loop {
        // Forward direction with increasing speed
        info!("Setting direction: forward");
        motor.set_direction(true);
        motor.start();
        
        for speed in (MIN_SPEED..=MAX_SPEED).step_by(20) {
            info!("Setting speed: {}%", speed);
            motor.set_speed(speed);
            
            // Get and display current state
            let state = motor.get_state();
            info!("Motor state: enabled={}, direction={}, duty_cycle={:?}, max_duty={:?}", 
                state.enabled, state.direction, state.duty_cycle, state.max_duty_cycle);
            
            Timer::after_millis(STEP_TIME_MS).await;
        }

        // Stop and wait
        info!("Stopping motor");
        motor.stop();
        Timer::after_millis(STEP_TIME_MS).await;

        // Reverse direction with decreasing speed
        info!("Setting direction: reverse");
        motor.set_direction(false);
        motor.start();
        
        for speed in (MIN_SPEED..=MAX_SPEED).rev().step_by(20) {
            info!("Setting speed: {}%", speed);
            motor.set_speed(speed);
            
            // Get and display current state
            let state = motor.get_state();
            info!("Motor state: enabled={}, direction={}, duty_cycle={:?}, max_duty={:?}", 
                state.enabled, state.direction, state.duty_cycle, state.max_duty_cycle);
            
            Timer::after_millis(STEP_TIME_MS).await;
        }

        // Stop and wait
        info!("Stopping motor");
        motor.stop();
        Timer::after_millis(STEP_TIME_MS).await;
    }
}
