#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use stm32::Board;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Match led_pin from board.toml (simplified for PC13)
    #[cfg(led_pin = "PC13")]
    let mut board = Board::new(p.PC13);

    #[cfg(not(led_pin = "PC13"))]
    compile_error!("Only PC13 is supported in this example; update board.toml");

    loop {
        info!("LED on");
        board.led_on();
        Timer::after_millis(500).await;

        info!("LED off");
        board.led_off();
        Timer::after_millis(500).await;
    }
}