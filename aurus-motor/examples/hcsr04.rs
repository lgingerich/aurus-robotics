#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Input, Output, Pull, Speed},
    peripherals::{PA0, PA1},
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Print program start
    info!("Starting capability-driven GPIO ultrasonic sensor control program...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Peripherals initialized");

    // Configure GPIO pins
    let mut trigger = Output::new(p.PA0, Level::Low, Speed::Low);
    let echo = Input::new(p.PA1, Pull::None);

    loop {
        if let Some(distance_cm) = measure_distance(&mut trigger, &echo).await {
            info!("Distance: {} cm", distance_cm);
        } else {
            warn!("Measurement timeout");
        }

        Timer::after(Duration::from_millis(80)).await;
    }
}

/// Measures the distance using HC-SR04.
/// Returns distance in cm or `None` if timeout occurs.
async fn measure_distance(trigger: &mut Output<'_>, echo: &Input<'_>) -> Option<f32> {
    // 1. Send 10us trigger pulse
    trigger.set_low();
    Timer::after(Duration::from_micros(2)).await;
    trigger.set_high();
    Timer::after(Duration::from_micros(10)).await;
    trigger.set_low();

    // 2. Wait for echo pin to go high (start of echo pulse)
    let start_time = embassy_time::Instant::now();
    let echo_timeout = Duration::from_millis(30);

    while echo.is_low() {
        if embassy_time::Instant::now() - start_time > echo_timeout {
            warn!("Timeout waiting for echo pulse to start");
            return None;
        }
    }

    let pulse_start_time = embassy_time::Instant::now();

    // 3. Wait for echo pin to go low (end of echo pulse)
    while echo.is_high() {
        if embassy_time::Instant::now() - pulse_start_time > echo_timeout {
            warn!("Timeout waiting for echo pulse to end (pulse too long)");
            return None;
        }
    }

    let pulse_end_time = embassy_time::Instant::now();

    // 4. Calculate duration
    let duration = pulse_end_time - pulse_start_time;
    let duration_us = duration.as_micros() as f32;

    // 5. Convert to cm: sound speed = 343 m/s = 0.0343 cm/us
    let distance_cm = (duration_us * 0.0343) / 2.0;

    // HC-SR04 recommended ranging cycle is >60ms to prevent interference

    Some(distance_cm)
}