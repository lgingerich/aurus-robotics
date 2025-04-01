// use embassy_stm32::timer::simple_pwm::SimplePwm;
// use embassy_stm32::timer::GeneralInstance4Channel;

// #[derive(Debug)]
// pub enum PwmError {
//     InvalidDutyCycle,
//     HardwareError,
//     NotEnabled,
// }

// /// Capability to generate PWM signals
// pub trait PwmOutput {
//     type Error;
    
//     /// Enables the PWM output
//     fn enable(&mut self) -> Result<(), Self::Error>;
    
//     /// Disables the PWM output
//     fn disable(&mut self) -> Result<(), Self::Error>;
    
//     /// Sets the duty cycle (0 to max_duty_cycle)
//     fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error>;
    
//     /// Returns the maximum duty cycle value
//     fn max_duty_cycle(&self) -> Result<u16, Self::Error>;
    
//     /// Returns true if the PWM output is enabled
//     fn is_enabled(&self) -> Result<bool, Self::Error>;
    
//     /// Sets the duty cycle as a percentage (0-100)
//     fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), Self::Error> {
//         let percent = percent.min(100) as u32;
//         let max_duty = self.max_duty_cycle()? as u32;
//         let duty = ((percent * max_duty) / 100) as u16;
//         self.set_duty_cycle(duty)
//     }
// }

// // For PWM channels
// impl<'a> PwmOutput for SimplePwm<'a, GeneralInstance4Channel> {
//     type Error = PwmError;
    
//     fn enable(&mut self) -> Result<(), Self::Error> {
//         self.enable();
//         Ok(())
//     }
    
//     fn disable(&mut self) -> Result<(), Self::Error> {
//         self.disable();
//         Ok(())
//     }
    
//     fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
//         let max_duty = self.max_duty_cycle()?;
//         if duty > max_duty {
//             return Err(PwmError::InvalidDutyCycle);
//         }
//         self.set_duty_cycle(duty);
//         Ok(())
//     }
    
//     fn max_duty_cycle(&self) -> Result<u16, Self::Error> {
//         Ok(self.max_duty_cycle())
//     }
    
//     fn is_enabled(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_enabled())
//     }
// }