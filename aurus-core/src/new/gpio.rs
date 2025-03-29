use core::convert::Infallible;
use embassy_stm32::gpio::{Pin, Output};

/// Capability to set a digital output state (high/low)
pub trait DigitalOutput {
    type Error;
    
    fn new(pin: Pin) -> Self;
    fn set_high(&mut self) -> Result<(), Self::Error>;
    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn is_set_high(&self) -> Result<bool, Self::Error>;
    fn is_set_low(&self) -> Result<bool, Self::Error>;
}

// For GPIO Output pins
impl<'a> DigitalOutput for Output<'a, Pin> {
    type Error = Infallible;

    fn new(pin: Pin) -> Self {
        Output::new(pin, Level::Low, Speed::Low)
    }
    
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
    
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
    
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }   
}