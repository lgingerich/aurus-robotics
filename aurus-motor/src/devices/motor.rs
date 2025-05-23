use crate::hardware::gpio::{DigitalOutput, GpioError};
use crate::hardware::pwm::{PwmError, PwmOutput};
use crate::traits::motor::{MotorControl, MotorState, SpeedControl};
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

pub struct Motor<D, S, E>
where
    D: DigitalOutput,
    S: PwmOutput,
    E: DigitalOutput,
{
    direction_pin: D,
    speed_control: Option<S>,
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
    pub fn new(mut direction_pin: D, speed_control: Option<S>, mut enable_pin: Option<E>) -> Self {
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
}

impl<D, S, E> MotorControl for Motor<D, S, E>
where
    D: DigitalOutput,
    S: PwmOutput,
    E: DigitalOutput,
    D::Error: Into<MotorError>,
    S::Error: Into<MotorError>,
    E::Error: Into<MotorError>,
{
    type Error = MotorError;
    type State = MotorState;

    fn start(&mut self) -> Result<(), Self::Error> {
        if let Some(ref mut pin) = self.enable_pin {
            pin.set_high().map_err(Into::into)?;
        }
        if let Some(ref mut pwm) = self.speed_control {
            pwm.enable().map_err(Into::into)?;
        }
        self.enabled = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        if let Some(ref mut pwm) = self.speed_control {
            pwm.disable().map_err(Into::into)?;
        }
        if let Some(ref mut pin) = self.enable_pin {
            pin.set_low().map_err(Into::into)?;
        }
        self.enabled = false;
        Ok(())
    }

    fn set_direction(&mut self, forward: bool) -> Result<(), Self::Error> {
        if forward {
            self.direction_pin.set_low().map_err(Into::into)?;
        } else {
            self.direction_pin.set_high().map_err(Into::into)?;
        }
        self.direction = forward;
        Ok(())
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        let max_duty = self
            .speed_control
            .as_mut()
            .map(|pwm| pwm.max_duty_cycle())
            .transpose()
            .map_err(Into::into)?;

        let speed = if let Some(max_duty) = max_duty {
            if max_duty > 0 {
                Some((self.current_duty as f32 / max_duty as f32).clamp(0.0, 1.0))
            } else {
                Some(0.0)
            }
        } else {
            None
        };

        Ok(MotorState {
            enabled: self.enabled,
            direction: self.direction,
            duty_cycle: if self.speed_control.is_some() {
                Some(self.current_duty)
            } else {
                None
            },
            speed,
            pwm_frequency: None,
            max_duty_cycle: max_duty,
        })
    }
}

impl<D, S, E> SpeedControl for Motor<D, S, E>
where
    D: DigitalOutput,
    S: PwmOutput,
    E: DigitalOutput,
    D::Error: Into<MotorError>,
    S::Error: Into<MotorError>,
    E::Error: Into<MotorError>,
{
    fn set_speed(&mut self, duty_cycle: u16) -> Result<(), Self::Error> {
        if !self.enabled {
            return Err(MotorError::InvalidState);
        }

        if let Some(ref mut pwm) = self.speed_control {
            pwm.set_duty_cycle(duty_cycle).map_err(Into::into)?;
            self.current_duty = duty_cycle;
        }
        Ok(())
    }

    fn set_speed_percent(&mut self, percent: u8) -> Result<(), Self::Error> {
        if !self.enabled {
            return Err(MotorError::InvalidState);
        }

        if let Some(ref mut pwm) = self.speed_control {
            let percent = percent.min(100);
            pwm.set_duty_cycle_percent(percent).map_err(Into::into)?;

            let max_duty = pwm.max_duty_cycle().map_err(Into::into)?;
            self.current_duty = ((percent as u32 * max_duty as u32) / 100) as u16;
        }
        Ok(())
    }
}
