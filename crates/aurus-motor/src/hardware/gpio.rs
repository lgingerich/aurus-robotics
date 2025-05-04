use core::convert::Infallible;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};

#[derive(Debug)]
pub enum GpioError {
    PinError,
    InvalidState,
}

/// Capability to set a digital output state (high/low)
pub trait DigitalOutput {
    type Error;

    /// Creates a new digital output instance
    fn new(pin: impl Pin) -> Self;

    /// Sets the output to high state
    fn set_high(&mut self) -> Result<(), Self::Error>;

    /// Sets the output to low state
    fn set_low(&mut self) -> Result<(), Self::Error>;

    /// Returns true if the output is set to high
    fn is_set_high(&self) -> Result<bool, Self::Error>;

    /// Returns true if the output is set to low
    fn is_set_low(&self) -> Result<bool, Self::Error>;

    /// Toggles the current state of the output
    fn toggle(&mut self) -> Result<(), Self::Error>;
}

// For GPIO Output pins
impl<'a> DigitalOutput for Output<'a> {
    type Error = Infallible;

    fn new(pin: impl Pin) -> Self {
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

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}
