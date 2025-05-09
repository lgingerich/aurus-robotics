A `no_std` Rust library for 2D differential-drive robot kinematics calculations. This crate provides structures and functions for:

*   Calculating robot pose (`x`, `y`, `θ`).
*   Forward kinematics (calculating chassis speeds from wheel speeds).
*   Inverse kinematics (calculating wheel speeds from chassis speeds).
*   Updating pose based on chassis speeds over a time delta (odometry).

## Features

*   **`no_std` compatible**: Suitable for embedded systems and environments without a standard library.
*   **Core kinematic calculations**: `Pose`, `Twist`, `WheelSpeeds`, `ChassisSpeeds`.
*   **Error handling**: Clear error types for invalid parameters or operations.
*   **Optional `serde` support**: For serialization and deserialization of kinematic types.

### Enabling `serde` support

To enable serialization/deserialization for the kinematic types, add the `serde` feature:

```toml
[dependencies]
aurus-kinematics = { version = "0.1.0", features = ["serde"] } # Replace with the latest version
```

## Basic Usage

Here's a simple example of how to use `aurus-kinematics` to update a robot's pose:

```rust
use aurus_kinematics::{DifferentialDrive, Pose, ChassisSpeeds, KinematicsError};

fn main() -> Result<(), KinematicsError> {
    // Robot parameters
    let wheel_radius = 0.05; // meters
    let axle_length = 0.3;   // meters

    let kinematics = DifferentialDrive::new(wheel_radius, axle_length)?;

    let mut current_pose = Pose::new(0.0, 0.0, 0.0); // (x, y, theta)
    let chassis_speeds = ChassisSpeeds::new(0.5, 0.0); // 0.5 m/s forward, 0 rad/s turning
    let dt = 0.1; // 0.1 seconds time step

    println!("Initial Pose: {}", current_pose);

    // Simulate 10 steps
    for i in 0..10 {
        current_pose = kinematics.update_pose(current_pose, chassis_speeds, dt)?;
        println!("Step {}: Pose: {}", i + 1, current_pose);
    }

    println!("Final Pose after {}s: {}", dt * 10.0, current_pose);
    Ok(())
}
```

Check out the `examples` directory for more usage patterns.
