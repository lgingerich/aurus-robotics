use aurus_kinematics::{ChassisSpeeds, DifferentialDrive, Pose, Twist};

// Calculates a new pose based on previous pose, applied twist, and delta time.
// This is a core function for odometry calculation using kinematics.
pub fn update_pose_from_kinematics(previous_pose: &Pose, applied_twist: &Twist, dt: f64) -> Pose {
    let kinematics_chassis_speeds = ChassisSpeeds {
        v: applied_twist.vx,
        omega: applied_twist.wz,
    };

    // Instantiate the kinematics handler from the crate
    // TODO: These values (wheel_radius, axle_length) should ideally come from configuration.
    let wheel_radius = 0.1; // meters
    let axle_length = 0.5; // meters
    let kinematics_handler = DifferentialDrive::new(wheel_radius, axle_length).unwrap();

    // Call the crate's update_pose function
    kinematics_handler
        .update_pose(*previous_pose, kinematics_chassis_speeds, dt)
        .expect("Failed to update pose")
} 