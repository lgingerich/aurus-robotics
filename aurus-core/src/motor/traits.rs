/// Represents the comprehensive state of a motor driver.
///
/// This structure encapsulates all the operational parameters of a motor driver,
/// providing a complete snapshot of its current state. It includes information about
/// the motor's operational status, control parameters, and configuration settings.
/// This allows for consistent state reporting across different motor driver implementations.
pub struct MotorDriverState {
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

/// Base trait that defines the fundamental interface for all motor drivers.
///
/// This trait provides essential motor control operations like starting,
/// stopping, and direction control.
pub trait MotorDriver {
    /// Configuration type for creating a new motor driver
    type Config: 'static; // Ensure static lifetime

    /// Creates a new instance of the motor driver.
    fn new(config: Self::Config) -> Self;

    /// Starts the motor.
    fn start(&mut self);

    /// Stops the motor.
    fn stop(&mut self);

    /// Cleans up the motor driver and stops the motor.
    fn cleanup(&mut self);

    /// Returns a complete snapshot of the motor driver's current state.
    ///
    /// This method provides a standardized way to access all operational parameters
    /// of the motor, regardless of the underlying implementation (GPIO, PWM, etc.).
    /// It enables consistent monitoring and control across different motor driver types.
    ///
    /// # Returns
    /// A `MotorDriverState` structure containing all current motor parameters including:
    /// - Enabled status
    /// - Duty cycle
    /// - Direction
    /// - Speed
    /// - PWM frequency (if applicable)
    /// - Maximum duty cycle value
    fn get_state(&mut self) -> MotorDriverState;

    /// Returns whether the motor is currently running.
    ///
    /// # Returns
    /// * `true` if the motor is running
    /// * `false` if the motor is stopped
    fn is_running(&mut self) -> bool;

    /// Sets the rotation direction of the motor.
    ///
    /// # Arguments
    /// * `forward` - `true` for forward rotation
    /// * `reverse` - `false` for reverse rotation
    fn set_direction(&mut self, forward: bool);

    /// Gets the current rotation direction of the motor.
    ///
    /// # Returns
    /// * `true` if rotating forward
    /// * `false` if rotating in reverse
    fn get_direction(&self) -> bool;
}

/// Extended trait for motors that support variable speed control.
///
/// This trait extends [`MotorDriver`] to add speed control capabilities.
pub trait SpeedControl: MotorDriver {
    /// Sets the motor speed.
    /// Note: Current implementation is a function of duty_cycle
    ///
    /// # Arguments
    /// * `duty_cycle` - Duty cycle value between 0 and max_duty_cycle
    fn set_speed(&mut self, duty_cycle: u8);

    /// Gets the current motor speed.
    ///
    /// # Returns
    /// Current speed value between 0 and max_duty_cycle
    fn get_current_speed(&mut self) -> u8;
}
