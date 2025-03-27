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

    /// Returns whether the motor is currently running.
    ///
    /// # Returns
    /// * `true` if the motor is running
    /// * `false` if the motor is stopped
    fn is_running(&self) -> bool;

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
    ///
    /// # Arguments
    /// * `speed` - Speed value between 0.0 and 1.0, where:
    ///   * 0.0 represents stopped
    ///   * 1.0 represents maximum speed
    fn set_speed(&mut self, speed: f32);

    /// Gets the current motor speed.
    ///
    /// # Returns
    /// Current speed value between 0.0 and 1.0
    fn get_speed(&self) -> f32;
}
