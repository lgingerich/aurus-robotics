#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PC13, Level::Low, Speed::Low);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_secs(1).await;
        
        info!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}