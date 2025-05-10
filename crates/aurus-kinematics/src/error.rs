#![warn(missing_docs)]

/// Error type for kinematics operations.
///
/// This enum encapsulates all possible errors that can occur during
/// kinematic calculations, such as invalid parameters or time deltas.
#[derive(Debug, PartialEq)]
pub enum KinematicsError {
    /// Error for invalid wheel radius.
    /// This variant is returned when a wheel radius is provided that is not positive.
    InvalidWheelRadius(&'static str),
    /// Error for invalid axle length.
    /// This variant is returned when an axle length is provided that is not positive.
    InvalidAxleLength(&'static str),
    /// Error for negative time delta.
    /// This variant is returned when a negative time delta is used for pose updates.
    NegativeTimeDelta(&'static str),
}

impl core::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            KinematicsError::InvalidWheelRadius(msg) => write!(f, "Invalid wheel radius: {}", msg),
            KinematicsError::InvalidAxleLength(msg) => write!(f, "Invalid axle length: {}", msg),
            KinematicsError::NegativeTimeDelta(msg) => write!(f, "Negative time delta: {}", msg),
        }
    }
}

impl core::error::Error for KinematicsError {}
