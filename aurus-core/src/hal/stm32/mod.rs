#![no_std]

use embassy_stm32::gpio::{Output, Level, Speed, Pin};
use embassy_stm32::Peripherals;

pub fn init_peripherals() -> Peripherals {
    embassy_stm32::init(Default::default())
}

// pub struct GpioOutput {
//     pin: Output<'static>,
// }

// pub trait Gpio {
//     /// Create a new GPIO output pin
//     fn new<P: Pin>(pin: P, level: Level, speed: Speed) -> Self;
    
//     /// Set the output level of the pin
//     fn set_level(&mut self, level: Level);
    
//     /// Get the current output level of the pin
//     fn get_level(&self) -> Level;

//     /// Toggle the output level of the pin
//     fn toggle(&mut self);
// }

// impl Gpio for GpioOutput {
//     fn new<P: Pin>(pin: P, level: Level, speed: Speed) -> Self {
//         Self { 
//             pin: Output::new(pin, level, speed)
//         }
//     }

//     fn set_level(&mut self, level: Level) {
//         self.pin.set_level(level);
//     }

//     fn get_level(&self) -> Level {
//         self.pin.get_output_level()
//     }

//     fn toggle(&mut self) {
//         self.pin.toggle();
//     }
// }

// pub trait Pwm {
//     fn set_duty_cycle {}
//     fn set_frequency {}
//     fn set_direction {}
// }


