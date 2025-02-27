#![no_std]

use embassy_stm32::gpio::{Output, Level, Speed, Pin};
use embassy_stm32::Peripherals;

pub fn init_peripherals() -> Peripherals {
    embassy_stm32::init(Default::default())
}

pub struct LedDriver {
    pub led: Output<'static>,
}

impl LedDriver {
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

pub struct MotorDriver {
    dir: Output<'static>,
    pwm: Output<'static>,
}

impl MotorDriver {
    pub fn new<P1: Pin, P2: Pin>(dir_pin: P1, pwm_pin: P2) -> Self {
        let dir = Output::new(dir_pin.degrade(), Level::High, Speed::Low); // Forward by default
        let pwm = Output::new(pwm_pin.degrade(), Level::Low, Speed::Low);  // Off by default
        Self { dir, pwm }
    }

    pub fn motor_on(&mut self) {
        self.pwm.set_high();  // PWM1 high = motor on (full speed)
    }

    pub fn motor_off(&mut self) {
        self.pwm.set_low();   // PWM1 low = motor off
    }

    pub fn set_direction(&mut self, forward: bool) {
        if forward {
            self.dir.set_high();
        } else {
            self.dir.set_low();
        }
    }
}

