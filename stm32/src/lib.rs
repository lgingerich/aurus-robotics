#![no_std]

use embassy_stm32::gpio::{Output, Level, Speed, Pin};

pub struct Board {
    pub led: Output<'static>,
}

impl Board {
    pub fn new<P: Pin>(pin: P) -> Self {
        let led = Output::new(pin, Level::Low, Speed::Low);
        Self { led }
    }

    pub fn led_on(&mut self) {
        self.led.set_high();
    }

    pub fn led_off(&mut self) {
        self.led.set_low();
    }
}