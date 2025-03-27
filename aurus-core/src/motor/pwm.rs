use crate::motor::traits::{MotorDriver, MotorDriverState, SpeedControl};
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed, OutputType},
    peripherals::{PA8, TIM1},
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};

#[cfg(test)]
use defmt_rtt as _; // global logger
#[cfg(test)]
use panic_probe as _; // panic handler

pub struct PwmMotorDriver {
    pwm: SimplePwm<'static, TIM1>,
    dir: Output<'static>,
}

impl MotorDriver for PwmMotorDriver {
    // TODO: This doesn't feel right to have TIM1 and PA8. 
    // TIM1_CH1 = PA8
    // Is it mandatory that I set the timer and pwm on the same pins?
    type Config = (
        TIM1,
        PA8, // PWM pin (Channel 1 for TIM1)
        AnyPin, // Direction pin
        u32
    );

    /// Create a new PwmMotorDriver
    fn new(config: Self::Config) -> Self {
        let (timer, pwm_pin, dir_pin, freq) = config;
        let freq = Hertz::hz(freq);

        // Create PWM pin first
        let ch1_pin = PwmPin::new_ch1(pwm_pin, OutputType::PushPull);
        
        // Create PWM with the pin
        let mut pwm = SimplePwm::new(timer, Some(ch1_pin), None, None, None, freq, Default::default());
        
        // Enable channel 1
        pwm.ch1().enable();

        // Create direction pin
        let dir = Output::new(dir_pin, Level::High, Speed::Low); // NOTE: Check these parameters

        Self { pwm, dir }
    }

    /// Start the motor
    fn start(&mut self) {
        self.pwm.ch1().set_duty_cycle(0);
    }

    /// Stop the motor
    fn stop(&mut self) {
        self.pwm.ch1().set_duty_cycle(0);
    }

    /// Cleanup the motor driver
    fn cleanup(&mut self) {
        self.pwm.ch1().set_duty_cycle(0); // Stop motor
        self.pwm.ch1().disable(); // Disable PWM
    }

    /// Get the current state of the motor driver
    fn get_state(&mut self) -> MotorDriverState {
        MotorDriverState {
            enabled: self.pwm.ch1().is_enabled(),
            direction: self.dir.is_set_high(),
            speed: None, // TODO: Add speed
            pwm_frequency: None, // TODO: Add frequency
            duty_cycle: Some(self.pwm.ch1().current_duty_cycle()),
            max_duty_cycle: Some(self.pwm.ch1().max_duty_cycle()),
        }
    }

    // TODO: Need to better define exactly what "running" means for this motor driver
    /// Check if the motor is running
    fn is_running(&mut self) -> bool {
        let ch = self.pwm.ch1();
        ch.is_enabled() && ch.current_duty_cycle() > 0
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

impl SpeedControl for PwmMotorDriver {
    // NOTE: This implementation is currently a function of duty_cycle_percent
    // Actual speed is a function of the motor, duty_cycle_percent, and voltage
    fn set_speed(&mut self, duty_cycle: u8) {
        self.pwm.ch1().set_duty_cycle_percent(duty_cycle);
    }

    fn get_current_speed(&mut self) -> u8 {
        duty_cycle_to_percent(self.pwm.ch1().current_duty_cycle(), self.pwm.ch1().max_duty_cycle())
    }
}

/// Convert the raw duty cycle value to a percentage (0-100)
pub fn duty_cycle_to_percent(current: u16, max: u16) -> u8 {
    assert!(current <= max, "Current duty cycle cannot exceed max duty cycle");
    let percent = (current as u32 * 100u32) / max as u32;
    percent as u8 // percent cannot exceed 100 (i.e. valid u8)
}
