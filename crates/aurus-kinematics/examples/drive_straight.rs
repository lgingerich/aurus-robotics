use aurus_kinematics::*;

fn main() {
    let wheel_radius = 0.1;
    let axle_length = 0.5;
    let kinematics_result = DifferentialDrive::new(wheel_radius, axle_length);

    let mut current_pose = Pose::new(0.0, 0.0, 0.0);
    let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1.0 m/s forward, 0 rad/s turning
    let dt = 0.1; // Time step in seconds
    let num_steps = 10;

    match kinematics_result {
        Ok(kinematics) => {
            println!("Initializing simulation...");
            println!("  Differential Drive Parameters:");
            println!("    Wheel Radius: {} m", kinematics.wheel_radius());
            println!("    Axle Length:  {} m", kinematics.axle_length());
            println!("  Initial State:");
            println!("    Pose:         {:?}", current_pose);
            println!("    Chassis Speeds: {:?}", chassis_speeds);
            println!("  Simulation Settings:");
            println!("    Time Step:    {} s", dt);
            println!("    Num Steps:    {}", num_steps);
            println!("
Simulating...");

            for i in 0..num_steps {
                match kinematics.update_pose(current_pose, chassis_speeds, dt) {
                    Ok(new_pose) => {
                        current_pose = new_pose;
                        println!("Step {:>2}: Pose: {}", i + 1, current_pose);
                    }
                    Err(e) => {
                        eprintln!("Error during simulation step {}: {:?}", i + 1, e);
                        break; // Stop loop on error
                    }
                }
            }

            println!("
Simulation complete.");
            println!("Final Pose: {:?}", current_pose);
        }
        Err(e) => {
            eprintln!("Failed to initialize kinematics: {:?}", e);
            eprintln!("Please ensure wheel_radius ({}) and axle_length ({}) are positive.", wheel_radius, axle_length);
        }
    }
}
