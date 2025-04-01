use embassy_stm32::timer::simple_pwm::{SimplePwm, PwmPin};
use embassy_stm32::timer::GeneralInstance4Channel;
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::OutputType;

#[derive(Debug)]
pub enum PwmError {
    InvalidDutyCycle,
    HardwareError,
    NotEnabled,
}

/// Configuration for PWM output
pub struct PwmConfig<T> {
    pub timer: T,
    pub frequency: u32,
}

/// Capability to generate PWM signals
pub trait PwmOutput {
    type Error;
    type Config;
    
    /// Creates a new PWM output instance
    fn new(config: Self::Config) -> Self;

    /// Enables the PWM output
    fn enable(&mut self) -> Result<(), Self::Error>;
    
    /// Disables the PWM output
    fn disable(&mut self) -> Result<(), Self::Error>;
    
    /// Sets the duty cycle (0 to max_duty_cycle)
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error>;
    
    /// Returns the maximum duty cycle value
    fn max_duty_cycle(&mut self) -> Result<u16, Self::Error>;
    
    /// Returns true if the PWM output is enabled
    fn is_enabled(&mut self) -> Result<bool, Self::Error>;
    
    /// Sets the duty cycle as a percentage (0-100)
    fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), Self::Error> {
        let percent = percent.min(100) as u32;
        let max_duty = self.max_duty_cycle()? as u32;
        let duty = ((percent * max_duty) / 100) as u16;
        self.set_duty_cycle(duty)
    }
}

// For PWM channels
impl<'a, T> PwmOutput for SimplePwm<'a, T> 
where T: GeneralInstance4Channel 
{
    type Error = PwmError;
    type Config = PwmConfig<T>;

    fn new(config: Self::Config) -> Self {
        SimplePwm::new(
            config.timer,
            None,
            None,
            None,
            None,
            Hertz::hz(config.frequency),
            Default::default()
        )
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.ch1().enable();
        Ok(())
    }
    
    fn disable(&mut self) -> Result<(), Self::Error> {
        self.ch1().disable();
        Ok(())
    }
    
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let max_duty = self.max_duty_cycle()?;
        if duty > max_duty {
            return Err(PwmError::InvalidDutyCycle);
        }
        self.ch1().set_duty_cycle(duty);
        Ok(())
    }
    
    fn max_duty_cycle(&mut self) -> Result<u16, Self::Error> {
        Ok(self.ch1().max_duty_cycle())
    }
    
    fn is_enabled(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ch1().is_enabled())
    }
}