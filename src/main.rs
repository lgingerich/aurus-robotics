mod robot;
mod hardware;
mod ros2_interface;

use crate::robot::{Robot, Position};

fn main() {
    // Create a new robot with mock hardware
    let mut robot = Robot::new();

    // Define where the red cube is
    let cube_position = Position {
        x: 1.0,
        y: 2.0,
        z: 0.0,
    };

    // Pick up the cube - this will now use mock hardware
    match robot.pick_up("red_cube", cube_position) {
        Ok(_) => println!("Successfully picked up the cube"),
        Err(e) => println!("Failed to pick up cube: {:?}", e),
    }

    // Print final state including sensor readings
    println!("Final robot state: {:?}", robot);
}