#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]
#![doc = "A `no_std` library for 2D differential-drive robot kinematics."]
#![doc = ""]
#![doc = "This crate provides structures and functions for calculating robot pose,"]
#![doc = "forward and inverse kinematics, and updating pose based on chassis or wheel speeds."]

use core::f64::consts::PI;
use core::fmt;
use libm::{cos, sin};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub mod error;
pub use error::KinematicsError;

/// A 2‑D pose `(x, y, θ)` in meters and radians (θ measured counter‑clockwise
/// from the x‑axis in the world frame).
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    /// World‑frame x position (m).
    pub x: f64,
    /// World‑frame y position (m).
    pub y: f64,
    /// Heading (rad), normalized to `[-PI, PI)`.
    pub theta: f64,
}

impl Pose {
    /// Construct a new pose.
    ///
    /// # Arguments
    ///
    /// * `x`: World-frame x position in meters.
    /// * `y`: World-frame y position in meters.
    /// * `theta`: Heading in radians.
    pub const fn new(x: f64, y: f64, theta: f64) -> Self {
        Pose { x, y, theta }
    }

    /// Normalize an angle to be within `[-PI, PI)`.
    ///
    /// Angles at `PI` will be normalized to `-PI`.
    ///
    /// # Arguments
    ///
    /// * `angle`: The angle in radians to normalize.
    ///
    /// # Returns
    ///
    /// The normalized angle in radians.
    pub fn normalize_angle(angle: f64) -> f64 {
        let a = angle % (2.0 * PI);
        if a >= PI {
            a - 2.0 * PI
        } else if a < -PI {
            a + 2.0 * PI
        } else {
            a
        }
    }
}

impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(x: {:.2}, y: {:.2}, θ: {:.2} rad)", self.x, self.y, self.theta)
    }
}

/// A twist expressed in the robot base frame.
/// A twist represents the linear and angular velocities of the robot.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist {
    /// Linear x velocity (m/s) in the robot's base frame.
    pub vx: f64,
    /// Angular z velocity (rad/s) around the robot's base frame z-axis.
    pub wz: f64,
}

impl Twist {
    /// Construct a new twist.
    ///
    /// # Arguments
    ///
    /// * `vx`: Linear velocity along the robot's x-axis (m/s).
    /// * `wz`: Angular velocity around the robot's z-axis (rad/s).
    pub const fn new(vx: f64, wz: f64) -> Self {
        Twist { vx, wz }
    }
}

impl fmt::Display for Twist {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(vx: {:.2} m/s, ωz: {:.2} rad/s)", self.vx, self.wz)
    }
}

/// Left and right wheel angular velocities.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelSpeeds {
    /// Left wheel angular velocity (rad/s).
    pub omega_l: f64,
    /// Right wheel angular velocity (rad/s).
    pub omega_r: f64,
}

impl WheelSpeeds {
    /// Construct wheel speeds.
    ///
    /// # Arguments
    ///
    /// * `omega_l`: Left wheel angular velocity (rad/s).
    /// * `omega_r`: Right wheel angular velocity (rad/s).
    pub const fn new(omega_l: f64, omega_r: f64) -> Self {
        WheelSpeeds { omega_l, omega_r }
    }
}

impl fmt::Display for WheelSpeeds {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(ωL: {:.2} rad/s, ωR: {:.2} rad/s)", self.omega_l, self.omega_r)
    }
}

/// Linear and angular chassis velocities.
/// These represent the overall motion of the robot's chassis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChassisSpeeds {
    /// Linear speed of the chassis center (m/s).
    pub v: f64,
    /// Angular speed of the chassis (rad/s).
    pub omega: f64,
}

impl ChassisSpeeds {
    /// Construct chassis speeds.
    ///
    /// # Arguments
    ///
    /// * `v`: Linear speed of the chassis center (m/s).
    /// * `omega`: Angular speed of the chassis (rad/s).
    pub const fn new(v: f64, omega: f64) -> Self {
        ChassisSpeeds { v, omega }
    }
}

impl fmt::Display for ChassisSpeeds {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "(v: {:.2} m/s, ω: {:.2} rad/s)", self.v, self.omega)
    }
}

/// Differential‑drive kinematics helper.
///
/// This struct encapsulates the physical parameters of a differential-drive robot
/// (wheel radius and axle length) and provides methods for kinematic calculations.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDrive {
    /// Wheel radius (m).
    wheel_radius: f64,
    /// Axle length (m).
    axle_length: f64,
}

impl DifferentialDrive {
    /// Construct a new differential‑drive kinematics helper.
    ///
    /// # Arguments
    ///
    /// * `wheel_radius`: The radius of the robot's wheels in meters.
    /// * `axle_length`: The distance between the centers of the two drive wheels in meters.
    ///
    /// # Errors
    ///
    /// Returns `Err(KinematicsError::InvalidWheelRadius)` if `wheel_radius` is not positive.
    /// Returns `Err(KinematicsError::InvalidAxleLength)` if `axle_length` is not positive.
    pub const fn new(wheel_radius: f64, axle_length: f64) -> Result<Self, KinematicsError> {
        if wheel_radius <= 0.0 {
            return Err(KinematicsError::InvalidWheelRadius(
                "must be positive",
            ));
        }
        if axle_length <= 0.0 {
            return Err(KinematicsError::InvalidAxleLength(
                "must be positive",
            ));
        }
        Ok(DifferentialDrive {
            wheel_radius,
            axle_length,
        })
    }

    /// Returns the wheel radius.
    pub fn wheel_radius(&self) -> f64 {
        self.wheel_radius
    }

    /// Returns the axle length.
    pub fn axle_length(&self) -> f64 {
        self.axle_length
    }

    /// Calculates the robot's chassis speeds (linear and angular velocity)
    /// from the wheel speeds. This is the forward kinematics problem.
    ///
    /// # Arguments
    ///
    /// * `wheel_speeds`: The measured or commanded angular velocities of the left and right wheels.
    ///
    /// # Returns
    ///
    /// The resulting linear and angular velocities of the robot chassis.
    pub fn forward_kinematics(&self, wheel_speeds: WheelSpeeds) -> ChassisSpeeds {
        let v_l = wheel_speeds.omega_l * self.wheel_radius;
        let v_r = wheel_speeds.omega_r * self.wheel_radius;

        let v = (v_r + v_l) / 2.0;
        let omega = (v_r - v_l) / self.axle_length;

        ChassisSpeeds::new(v, omega)
    }

    /// Calculates the required wheel speeds to achieve the given chassis speeds.
    /// This is the inverse kinematics problem.
    ///
    /// # Arguments
    ///
    /// * `chassis_speeds`: The desired linear and angular velocities of the robot chassis.
    ///
    /// # Returns
    ///
    /// The required angular velocities for the left and right wheels.
    pub fn inverse_kinematics(&self, chassis_speeds: ChassisSpeeds) -> WheelSpeeds {
        let v_r = chassis_speeds.v + chassis_speeds.omega * (self.axle_length / 2.0);
        let v_l = chassis_speeds.v - chassis_speeds.omega * (self.axle_length / 2.0);

        let omega_r = v_r / self.wheel_radius;
        let omega_l = v_l / self.wheel_radius;

        WheelSpeeds::new(omega_l, omega_r)
    }

    /// Updates the robot's pose given its current pose, chassis speeds, and time delta.
    ///
    /// This method performs odometry, integrating the chassis speeds over the time delta `dt`
    /// to estimate the new pose. It assumes constant chassis speeds over the interval `dt`.
    /// The final heading is normalized to `[-PI, PI)`.
    ///
    /// # Arguments
    ///
    /// * `current_pose`: The robot's current pose `(x, y, theta)`.
    /// * `chassis_speeds`: The robot's current linear and angular chassis speeds.
    /// * `dt`: The time delta in seconds over which the speeds are applied.
    ///
    /// # Errors
    ///
    /// Returns `Err(KinematicsError::NegativeTimeDelta)` if `dt` is negative.
    ///
    /// # Returns
    ///
    /// The robot's new estimated pose.
    pub fn update_pose(
        &self,
        current_pose: Pose,
        chassis_speeds: ChassisSpeeds,
        dt: f64,
    ) -> Result<Pose, KinematicsError> {
        if dt < 0.0 {
            return Err(KinematicsError::NegativeTimeDelta(
                "must be non-negative",
            ));
        }

        let delta_x = chassis_speeds.v * cos(current_pose.theta) * dt;
        let delta_y = chassis_speeds.v * sin(current_pose.theta) * dt;
        let delta_theta = chassis_speeds.omega * dt;

        Ok(Pose {
            x: current_pose.x + delta_x,
            y: current_pose.y + delta_y,
            theta: Pose::normalize_angle(current_pose.theta + delta_theta),
        })
    }

    /// Convenience function to update pose directly from wheel speeds and dt.
    ///
    /// This method first calculates chassis speeds using `forward_kinematics` and then
    /// calls `update_pose`.
    ///
    /// # Arguments
    ///
    /// * `current_pose`: The robot's current pose `(x, y, theta)`.
    /// * `wheel_speeds`: The measured or commanded angular velocities of the left and right wheels.
    /// * `dt`: The time delta in seconds over which the speeds are applied.
    ///
    /// # Errors
    ///
    /// Returns `Err(KinematicsError::NegativeTimeDelta)` if `dt` is negative (propagated from `update_pose`).
    ///
    /// # Returns
    ///
    /// The robot's new estimated pose.
    pub fn update_pose_from_wheel_speeds(
        &self,
        current_pose: Pose,
        wheel_speeds: WheelSpeeds,
        dt: f64,
    ) -> Result<Pose, KinematicsError> {
        let chassis_speeds = self.forward_kinematics(wheel_speeds);
        self.update_pose(current_pose, chassis_speeds, dt)
    }
}

impl fmt::Display for DifferentialDrive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DifferentialDrive (r: {:.2} m, L: {:.2} m)", self.wheel_radius, self.axle_length)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const EPSILON: f64 = 1e-6;

    #[test]
    fn test_pose_normalization() {
        assert!((Pose::normalize_angle(0.0) - 0.0).abs() < EPSILON);
        assert!((Pose::normalize_angle(PI) - (-PI)).abs() < EPSILON); // PI should map to -PI for [-PI, PI)
        assert!((Pose::normalize_angle(PI - EPSILON) - (PI - EPSILON)).abs() < EPSILON);
        assert!((Pose::normalize_angle(-PI) - -PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(3.0 * PI) - (-PI)).abs() < EPSILON); // 3*PI maps to PI, then to -PI
        assert!((Pose::normalize_angle(2.5 * PI) - 0.5 * PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(-2.5 * PI) - -0.5 * PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(-3.0 * PI) - -PI).abs() < EPSILON);
    }

    #[test]
    fn test_kinematics_constructor() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        assert_eq!(kinematics.wheel_radius, 0.1);
        assert_eq!(kinematics.axle_length, 0.5);
        assert_eq!(kinematics.wheel_radius(), 0.1); // Test getter
        assert_eq!(kinematics.axle_length(), 0.5);  // Test getter
    }

    #[test]
    // #[should_panic] // No longer panics, returns Err
    fn test_constructor_invalid_radius() {
        let result = DifferentialDrive::new(0.0, 0.5);
        assert!(matches!(result, Err(KinematicsError::InvalidWheelRadius("must be positive"))));
        let result_negative = DifferentialDrive::new(-0.1, 0.5);
        assert!(matches!(result_negative, Err(KinematicsError::InvalidWheelRadius("must be positive"))));
    }

    #[test]
    // #[should_panic] // No longer panics, returns Err
    fn test_constructor_invalid_axle_length() {
        let result = DifferentialDrive::new(0.1, 0.0);
        assert!(matches!(result, Err(KinematicsError::InvalidAxleLength("must be positive"))));
        let result_negative = DifferentialDrive::new(0.1, -0.5);
        assert!(matches!(result_negative, Err(KinematicsError::InvalidAxleLength("must be positive"))));
    }

    #[test]
    fn test_forward_kinematics_straight() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(10.0, 10.0); // Both wheels 10 rad/s
        // v_l = 10 * 0.1 = 1 m/s
        // v_r = 10 * 0.1 = 1 m/s
        // v = (1 + 1) / 2 = 1 m/s
        // omega = (1 - 1) / 0.5 = 0 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 1.0).abs() < EPSILON);
        assert!((chassis_speeds.omega - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_forward_kinematics_pivot_turn() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(-5.0, 5.0); // Left -5 rad/s, Right 5 rad/s
        // v_l = -5 * 0.1 = -0.5 m/s
        // v_r = 5 * 0.1 = 0.5 m/s
        // v = (0.5 + (-0.5)) / 2 = 0 m/s
        // omega = (0.5 - (-0.5)) / 0.5 = 1 / 0.5 = 2 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 0.0).abs() < EPSILON);
        assert!((chassis_speeds.omega - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_forward_kinematics_gentle_turn() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(5.0, 10.0); // Left 5 rad/s, Right 10 rad/s
        // v_l = 5 * 0.1 = 0.5 m/s
        // v_r = 10 * 0.1 = 1.0 m/s
        // v = (1.0 + 0.5) / 2 = 0.75 m/s
        // omega = (1.0 - 0.5) / 0.5 = 0.5 / 0.5 = 1 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 0.75).abs() < EPSILON);
        assert!((chassis_speeds.omega - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_inverse_kinematics_straight() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward, 0 rad/s
        // v_r = 1.0 + (0.0 * 0.5) / 2.0 = 1.0
        // v_l = 1.0 - (0.0 * 0.5) / 2.0 = 1.0
        // omega_r = 1.0 / 0.1 = 10.0 rad/s
        // omega_l = 1.0 / 0.1 = 10.0 rad/s
        let wheel_speeds = kinematics.inverse_kinematics(chassis_speeds);
        assert!((wheel_speeds.omega_l - 10.0).abs() < EPSILON);
        assert!((wheel_speeds.omega_r - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_inverse_kinematics_pivot_turn() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let chassis_speeds = ChassisSpeeds::new(0.0, 2.0); // 0 m/s, 2 rad/s
        // v_r = 0.0 + (2.0 * 0.5) / 2.0 = 0.0 + 1.0 / 2.0 = 0.5
        // v_l = 0.0 - (2.0 * 0.5) / 2.0 = 0.0 - 1.0 / 2.0 = -0.5
        // omega_r = 0.5 / 0.1 = 5.0 rad/s
        // omega_l = -0.5 / 0.1 = -5.0 rad/s
        let wheel_speeds = kinematics.inverse_kinematics(chassis_speeds);
        assert!((wheel_speeds.omega_l - (-5.0)).abs() < EPSILON);
        assert!((wheel_speeds.omega_r - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_inverse_kinematics_gentle_turn() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let chassis_speeds = ChassisSpeeds::new(0.75, 1.0); // 0.75 m/s, 1.0 rad/s
        // v_r = 0.75 + (1.0 * 0.5) / 2.0 = 0.75 + 0.25 = 1.0
        // v_l = 0.75 - (1.0 * 0.5) / 2.0 = 0.75 - 0.25 = 0.5
        // omega_r = 1.0 / 0.1 = 10.0 rad/s
        // omega_l = 0.5 / 0.1 = 5.0 rad/s
        let wheel_speeds = kinematics.inverse_kinematics(chassis_speeds);
        assert!((wheel_speeds.omega_l - 5.0).abs() < EPSILON);
        assert!((wheel_speeds.omega_r - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_straight_no_rotation() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(0.0, 0.0, 0.0); // Facing along X-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward, 0 rad/s
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 1*cos(0)*1 = 1
        //           y = 0 + 1*sin(0)*1 = 0
        //           theta = 0 + 0*1 = 0
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_straight_with_initial_rotation() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(1.0, 1.0, PI / 2.0); // At (1,1), facing along Y-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward, 0 rad/s
        let dt = 2.0; // 2 seconds

        // Expected: x = 1 + 1*cos(PI/2)*2 = 1 + 0 = 1
        //           y = 1 + 1*sin(PI/2)*2 = 1 + 2 = 3
        //           theta = PI/2 + 0*2 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 3.0).abs() < EPSILON);
        assert!((new_pose.theta - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_straight_negative_x_axis() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(0.0, 0.0, PI); // Facing along -X-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 1*cos(PI)*1 = 0 - 1 = -1
        //           y = 0 + 1*sin(PI)*1 = 0 + 0 = 0
        //           theta = PI + 0*1 = PI (normalized to -PI)
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        assert!((new_pose.x - (-1.0)).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - (-PI)).abs() < EPSILON); // PI normalizes to -PI
    }

    #[test]
    fn test_update_pose_straight_negative_y_axis() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(0.0, 0.0, -PI / 2.0); // Facing along -Y-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 1*cos(-PI/2)*1 = 0 + 0 = 0
        //           y = 0 + 1*sin(-PI/2)*1 = 0 - 1 = -1
        //           theta = -PI/2 + 0*1 = -PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        assert!((new_pose.x - 0.0).abs() < EPSILON);
        assert!((new_pose.y - (-1.0)).abs() < EPSILON);
        assert!((new_pose.theta - (-PI / 2.0)).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_pivot_turn_no_translation() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let chassis_speeds = ChassisSpeeds::new(0.0, PI / 2.0); // 0 m/s, PI/2 rad/s (90 deg/s)
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 0*cos(0)*1 = 0
        //           y = 0 + 0*sin(0)*1 = 0
        //           theta = 0 + (PI/2)*1 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        assert!((new_pose.x - 0.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_combined_motion() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(1.0, 2.0, PI / 4.0); // At (1,2), 45 deg
        let chassis_speeds = ChassisSpeeds::new(1.0, PI / 2.0); // 1 m/s, PI/2 rad/s
        let dt = 0.5; // 0.5 seconds

        // Expected intermediate values:
        // delta_x = 1.0 * cos(PI/4) * 0.5 = 1.0 * (sqrt(2)/2) * 0.5 = sqrt(2)/4 approx 0.35355
        // delta_y = 1.0 * sin(PI/4) * 0.5 = 1.0 * (sqrt(2)/2) * 0.5 = sqrt(2)/4 approx 0.35355
        // delta_theta = (PI/2) * 0.5 = PI/4
        //
        // x_new = 1.0 + 0.35355 = 1.35355
        // y_new = 2.0 + 0.35355 = 2.35355
        // theta_new = PI/4 + PI/4 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt).unwrap();
        let expected_x = 1.0 + (2.0_f64.sqrt() / 4.0);
        let expected_y = 2.0 + (2.0_f64.sqrt() / 4.0);
        let expected_theta = PI / 2.0;

        assert!((new_pose.x - expected_x).abs() < EPSILON);
        assert!((new_pose.y - expected_y).abs() < EPSILON);
        assert!((new_pose.theta - expected_theta).abs() < EPSILON);
    }

    #[test]
    // #[should_panic] // No longer panics, returns Err
    fn test_update_pose_negative_dt() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap();
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0);
        let result = kinematics.update_pose(current_pose, chassis_speeds, -0.1);
        assert!(matches!(result, Err(KinematicsError::NegativeTimeDelta("must be non-negative"))));
    }

    #[test]
    fn test_update_pose_from_wheel_speeds_straight() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let wheel_speeds = WheelSpeeds::new(10.0, 10.0); // Both wheels 10 rad/s => v=1m/s, omega=0rad/s
        let dt = 1.0;

        let new_pose = kinematics.update_pose_from_wheel_speeds(current_pose, wheel_speeds, dt).unwrap();
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_from_wheel_speeds_pivot_turn() {
        let kinematics = DifferentialDrive::new(0.1, 0.5).unwrap(); // r=0.1m, L=0.5m
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let wheel_speeds = WheelSpeeds::new(-PI, PI); // Left -PI rad/s, Right PI rad/s
        // v_l = -PI * 0.1 = -0.1*PI
        // v_r = PI * 0.1 = 0.1*PI
        // v = 0 m/s
        // omega = (0.1*PI - (-0.1*PI)) / 0.5 = (0.2*PI) / 0.5 = 0.4*PI rad/s
        let dt = 1.0; // 1 second
        // Expected theta_new = 0 + 0.4*PI*1 = 0.4*PI

        let new_pose = kinematics.update_pose_from_wheel_speeds(current_pose, wheel_speeds, dt).unwrap();
        assert!((new_pose.x - 0.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - (0.4 * PI)).abs() < EPSILON);
    }
}
