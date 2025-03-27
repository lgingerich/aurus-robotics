use crate::motor::traits::MotorDriver;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};

pub struct GpioMotorDriver {
    enable: Output<'static>,
    dir: Output<'static>,
}

impl MotorDriver for GpioMotorDriver {
    type Config = (
        AnyPin,
        AnyPin,
    );

    /// Create a new GpioMotorDriver
    fn new(config: Self::Config) -> Self {
        let (enable_pin, dir_pin) = config;
        // Initialize with motor stopped and forward direction
        let enable = Output::new(enable_pin, Level::Low, Speed::Low); // NOTE: On the motor driver board, the pins are named "DIR" and "PWM". Should I just call this "pwm" instead of "enable"?
        let dir = Output::new(dir_pin, Level::High, Speed::Low);

        Self { enable, dir }
    }
    
    // NOTE: Do I want this to always start at High?
    /// Start the motor
    fn start(&mut self) {
        self.enable.set_level(Level::High);
    }

    /// Stop the motor
    fn stop(&mut self) {
        self.enable.set_level(Level::Low);
    }

    /// Check if the motor is running
    fn is_running(&self) -> bool {
        self.enable.is_set_high()
    }

    /// Set the direction of the motor (true = forward, false = reverse)
    fn set_direction(&mut self, forward: bool) {
        if forward {
            self.dir.set_high();
        } else {
            self.dir.set_low();
        }
    }

    /// Get the current direction of the motor (true = forward, false = reverse)
    fn get_direction(&self) -> bool {
        self.dir.is_set_high()
    }
}