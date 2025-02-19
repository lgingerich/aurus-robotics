use embassy_stm32::{
    gpio::{Output, Level, Speed},
    init,
    Config
};

pub struct Board {
    pub led: Output<'static>,
}

impl Board {
    pub fn new() -> Self {
        // Initialize the peripherals with default config
        let p = init(Config::default());

        let led = Output::new(p.PC13, Level::High, Speed::Low);
        Self { led }
    }

    pub fn set_low(&mut self) {
        self.led.set_low();
    }

    pub fn set_high(&mut self) {
        self.led.set_high();
    }
}