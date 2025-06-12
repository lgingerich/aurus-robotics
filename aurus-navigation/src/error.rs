//! This module defines the error types used by the `aurus-navigation` crate.

#![warn(missing_docs)]

/// Error type for navigation operations.
///
/// This enum encapsulates all possible errors that can occur during
/// navigation operations, such as invalid map parameters or out-of-bounds access.
#[derive(Debug, PartialEq)]
pub enum NavigationError {
    /// Error for invalid map resolution.
    /// This variant is returned when a map resolution is provided that is not positive.
    InvalidResolution(&'static str),
    /// Error for invalid map dimensions.
    /// This variant is returned when map width or height is zero.
    InvalidDimensions(&'static str),
    /// Error for out-of-bounds access.
    /// This variant is returned when attempting to access map cells outside the valid range.
    OutOfBounds(&'static str),
    /// Error for invalid world coordinates.
    /// This variant is returned when world coordinates are outside the map bounds.
    InvalidWorldCoordinates(&'static str),
}

impl core::fmt::Display for NavigationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            NavigationError::InvalidResolution(msg) => write!(f, "Invalid map resolution: {}", msg),
            NavigationError::InvalidDimensions(msg) => write!(f, "Invalid map dimensions: {}", msg),
            NavigationError::OutOfBounds(msg) => write!(f, "Map access out of bounds: {}", msg),
            NavigationError::InvalidWorldCoordinates(msg) => {
                write!(f, "Invalid world coordinates: {}", msg)
            }
        }
    }
}

impl core::error::Error for NavigationError {}
