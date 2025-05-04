/// Represents the comprehensive state of a motor driver.
///
/// This structure encapsulates all the operational parameters of a motor driver,
/// providing a complete snapshot of its current state. It includes information about
/// the motor's operational status, control parameters, and configuration settings.
/// This allows for consistent state reporting across different motor driver implementations.
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    /// Whether the motor is enabled and receiving power.
    pub enabled: bool,
    /// The current direction of the motor rotation.
    /// - `true` represents forward rotation
    /// - `false` represents reverse rotation
    pub direction: bool,
    /// The current duty cycle of the motor (for PWM control).
    /// Typically ranges from 0 (completely off) to a maximum value defined by the controller.
    pub duty_cycle: Option<u16>,
    /// Current speed as a normalized value between 0.0 and 1.0.
    /// - 0.0 represents stopped
    /// - 1.0 represents maximum speed
    pub speed: Option<f32>,
    /// The PWM frequency in Hz (when applicable).
    /// - `None` for non-PWM drivers
    /// - `Some(frequency)` for PWM-based drivers
    pub pwm_frequency: Option<u32>,
    /// Maximum possible duty cycle value.
    /// This is hardware/implementation specific and helps interpret the duty_cycle field.
    pub max_duty_cycle: Option<u16>,
}

/// Core trait defining the interface for motor control
pub trait MotorControl {
    type Error;
    type State;

    /// Start the motor
    fn start(&mut self) -> Result<(), Self::Error>;

    /// Stop the motor
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Set the motor direction
    /// - true for forward
    /// - false for reverse
    fn set_direction(&mut self, forward: bool) -> Result<(), Self::Error>;

    /// Get the current state of the motor
    fn get_state(&mut self) -> Result<Self::State, Self::Error>;
}

/// Extended trait for motors that support speed control
pub trait SpeedControl: MotorControl {
    /// Set the motor speed using duty cycle
    fn set_speed(&mut self, duty_cycle: u16) -> Result<(), Self::Error>;

    /// Set the motor speed as a percentage (0-100)
    fn set_speed_percent(&mut self, percent: u8) -> Result<(), Self::Error>;
}
