#![no_std]
#![no_main]

use aurus_core::{
    devices::motor::Motor,
    traits::motor::{MotorControl, SpeedControl},
};
use embassy_stm32::{
    gpio::{Level, Output, OutputType, Speed},
    peripherals::{PA8, PA9, TIM1},
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};

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
    info!("Starting capability-driven PWM motor control program...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Peripherals initialized");

    // Configure PWM pin (PA8 - TIM1_CH1)
    let pwm_pin = p.PA8;
    let ch1_pin = PwmPin::new_ch1(pwm_pin, OutputType::PushPull);

    // Configure direction pin (PA0)
    let dir_pin: Output<'static> = Output::new(p.PA0, Level::Low, Speed::Low);

    // Create PWM instance
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        None,
        None,
        None,
        Hertz::hz(PWM_FREQ),
        Default::default(),
    );

    // Create motor instance
    let mut motor = Motor::<Output<'static>, SimplePwm<'static, TIM1>, Output<'static>>::new(
        dir_pin, Some(pwm), None,
    );
    info!("Motor initialized with PWM capabilities");

    info!("Entering main control loop");
    loop {
        // Forward direction with increasing speed
        info!("Setting direction: forward");
        motor.set_direction(true).unwrap();
        motor.start().unwrap();

        for speed in (MIN_SPEED..=MAX_SPEED).step_by(20) {
            info!("Setting speed: {}%", speed);
            motor.set_speed_percent(speed).unwrap();

            // Get and display current state
            let state = motor.get_state().unwrap();
            info!(
                "Motor state: enabled={}, direction={}, duty_cycle={:?}, max_duty={:?}",
                state.enabled, state.direction, state.duty_cycle, state.max_duty_cycle
            );

            Timer::after_millis(STEP_TIME_MS).await;
        }

        // Stop and wait
        info!("Stopping motor");
        motor.stop().unwrap();
        Timer::after_millis(STEP_TIME_MS).await;

        // Reverse direction with decreasing speed
        info!("Setting direction: reverse");
        motor.set_direction(false).unwrap();
        motor.start().unwrap();

        for speed in (MIN_SPEED..=MAX_SPEED).rev().step_by(20) {
            info!("Setting speed: {}%", speed);
            motor.set_speed_percent(speed).unwrap();

            // Get and display current state
            let state = motor.get_state().unwrap();
            info!(
                "Motor state: enabled={}, direction={}, duty_cycle={:?}, max_duty={:?}",
                state.enabled, state.direction, state.duty_cycle, state.max_duty_cycle
            );

            Timer::after_millis(STEP_TIME_MS).await;
        }

        // Stop and wait
        info!("Stopping motor");
        motor.stop().unwrap();
        Timer::after_millis(STEP_TIME_MS).await;
    }
}
