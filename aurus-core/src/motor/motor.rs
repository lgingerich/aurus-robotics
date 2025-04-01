use crate::motor::gpio::{DigitalOutput, GpioError};
use crate::motor::pwm::{PwmError, PwmOutput};
use core::convert::Infallible;

#[derive(Debug)]
pub enum MotorError {
    GpioError(GpioError),
    PwmError(PwmError),
    InvalidState,
}

impl From<GpioError> for MotorError {
    fn from(err: GpioError) -> Self {
        MotorError::GpioError(err)
    }
}

impl From<PwmError> for MotorError {
    fn from(err: PwmError) -> Self {
        MotorError::PwmError(err)
    }
}

impl From<Infallible> for MotorError {
    fn from(err: Infallible) -> Self {
        match err {}
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub enabled: bool,
    pub direction: bool,
    pub duty_cycle: Option<u16>,
    pub speed: Option<f32>,
    pub pwm_frequency: Option<u32>,
    pub max_duty_cycle: Option<u16>,
}

pub struct Motor<D, S, E>
where
    D: DigitalOutput,
    S: PwmOutput,
    E: DigitalOutput,
{
    direction_pin: D,
    speed_control: S,
    enable_pin: Option<E>,
    enabled: bool,
    direction: bool,
    current_duty: u16,
}

impl<D, S, E> Motor<D, S, E>
where
    D: DigitalOutput,
    S: PwmOutput,
    E: DigitalOutput,
    D::Error: Into<MotorError>,
    S::Error: Into<MotorError>,
    E::Error: Into<MotorError>,
{
    pub fn new(mut direction_pin: D, speed_control: S, mut enable_pin: Option<E>) -> Self {
        let _ = direction_pin.set_low();
        if let Some(ref mut pin) = enable_pin {
            let _ = pin.set_low();
        }

        Self {
            direction_pin,
            speed_control,
            enable_pin,
            enabled: false,
            direction: true,
            current_duty: 0,
        }
    }

    pub fn start(&mut self) -> Result<(), MotorError> {
        if let Some(ref mut pin) = self.enable_pin {
            pin.set_high().map_err(Into::into)?;
        }
        self.speed_control.enable().map_err(Into::into)?;
        self.enabled = true;
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), MotorError> {
        self.speed_control.disable().map_err(Into::into)?;
        if let Some(ref mut pin) = self.enable_pin {
            pin.set_low().map_err(Into::into)?;
        }
        self.enabled = false;
        Ok(())
    }

    pub fn set_direction(&mut self, forward: bool) -> Result<(), MotorError> {
        if forward {
            self.direction_pin.set_low().map_err(Into::into)?;
        } else {
            self.direction_pin.set_high().map_err(Into::into)?;
        }
        self.direction = forward;
        Ok(())
    }

    pub fn set_speed(&mut self, duty_cycle: u16) -> Result<(), MotorError> {
        if !self.enabled {
            return Err(MotorError::InvalidState);
        }

        self.speed_control
            .set_duty_cycle(duty_cycle)
            .map_err(Into::into)?;
        self.current_duty = duty_cycle;
        Ok(())
    }

    pub fn set_speed_percent(&mut self, percent: u8) -> Result<(), MotorError> {
        if !self.enabled {
            return Err(MotorError::InvalidState);
        }

        let percent = percent.min(100);
        self.speed_control
            .set_duty_cycle_percent(percent)
            .map_err(Into::into)?;

        let max_duty = self.speed_control.max_duty_cycle().map_err(Into::into)?;
        self.current_duty = ((percent as u32 * max_duty as u32) / 100) as u16;
        Ok(())
    }

    pub fn get_state(&mut self) -> Result<MotorState, MotorError> {
        let max_duty = self.speed_control.max_duty_cycle().map_err(Into::into)?;
        let speed = if max_duty > 0 {
            Some((self.current_duty as f32 / max_duty as f32).clamp(0.0, 1.0))
        } else {
            Some(0.0)
        };

        Ok(MotorState {
            enabled: self.enabled,
            direction: self.direction,
            duty_cycle: Some(self.current_duty),
            speed,
            pwm_frequency: None,
            max_duty_cycle: Some(max_duty),
        })
    }
}
