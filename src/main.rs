#![no_std]
#![no_main]

mod stm32;

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use crate::stm32::Board;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut led = Board::new();

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_millis(500).await;

        info!("led off!");
        led.set_low();
        Timer::after_millis(500).await;
    }
}