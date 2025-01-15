#[derive(Debug, Clone)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone)]
pub struct Orientation {
    pub roll: f64,  // rotation around x-axis
    pub pitch: f64, // rotation around y-axis
    pub yaw: f64,   // rotation around z-axis
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub position: Position,
    pub orientation: Orientation,
    pub gripper_open: bool,
    pub holding_object: Option<String>,
}

#[derive(Debug)]
pub enum RobotError {
    AlreadyHolding(String),
}

impl Robot {
    pub fn new() -> Self {
        Self {
            position: Position { x: 0.0, y: 0.0, z: 0.0 },
            orientation: Orientation { roll: 0.0, pitch: 0.0, yaw: 0.0 },
            gripper_open: true,
            holding_object: None,
        }
    }

    pub fn move_to(&mut self, target: Position) -> Result<(), RobotError> {
        println!("Moving from {:?} to {:?}", self.position, target);
        self.position = target;
        Ok(())
    }

    pub fn grip(&mut self, open: bool) -> Result<(), RobotError> {
        self.gripper_open = open;
        println!("Gripper is now {}", if open { "open" } else { "closed" });
        Ok(())
    }

    pub fn pick_up(&mut self, object: &str, object_position: Position) -> Result<(), RobotError> {
        if let Some(held) = &self.holding_object {
            return Err(RobotError::AlreadyHolding(held.clone()));
        }

        println!("Picking up {}", object);
        
        // Move above object
        let approach = Position {
            x: object_position.x,
            y: object_position.y,
            z: object_position.z + 0.1,
        };
        self.move_to(approach.clone())?;
        self.move_to(object_position)?;
        
        // Close gripper
        self.grip(false)?;
        
        // Store what we're holding
        self.holding_object = Some(object.to_string());
        
        // Move up
        self.move_to(approach)?;
        Ok(())
    }
}