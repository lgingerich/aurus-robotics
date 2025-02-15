#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();

    for _ in 0..10 {
        // Turn LED on
        led.set_low();
        cortex_m::asm::delay(2_000_000);
        // Turn LED off
        led.set_high();
        cortex_m::asm::delay(2_000_000);
    }

    // After blinking, enter an infinite loop doing nothing
    loop {
        led.set_high();  // Keep LED off after blinking
        cortex_m::asm::delay(2_000_000);
    }
}
