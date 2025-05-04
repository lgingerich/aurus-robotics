use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Debug, Clone)]
pub enum NavStatus {
    Idle,
    Navigating { goal_id: Uuid },
    Stuck,
    Planning,
    Error(String),
}

#[derive(Debug, Clone)]
pub struct BatteryState {
    pub level: f32,
    pub is_charging: bool,
}

#[derive(Debug, Clone)]
pub enum RobotMode {
    Initializing, // The robot is starting up. It's performing self-checks, calibrating sensors, loading software, connecting to the network, and verifying its system health.
    Ready, // The robot is powered on, healthy, knows its location, and is ready to receive commands or missions. It's not actively moving or performing a task.
    Navigating, // The robot has been given a destination and is actively planning a path and moving towards it while avoiding obstacles.
    ExecutingTask, // The robot has reached a location and is performing a specific non-navigation task.
    ShuttingDown, // The robot is performing its sequence to safely power off. This might involve parking in a specific location, saving state, shutting down systems gracefully.
    PoweredOff, // The robot is powered off and cannot be used.

    Paused, // The robot's current activity has been temporarily halted. It retains its current goal and context.
    Charging, // The robot is docked or connected to a charging station and is actively recharging its battery.
    Error, // The robot has encountered an error and is unable to continue its normal operation.
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SystemStatus {
    Nominal,
    Warning,
    Critical,
    Offline,
}


#[derive(Debug, Clone)]
pub struct RobotState {
    pub mode: RobotMode,
    pub system_status: SystemStatus,
    pub current_task_id: Option<Uuid>,
    pub pose: Option<Pose>,
    pub battery_state: Option<BatteryState>,
    pub nav_status: Option<NavStatus>,
    pub active_errors: Vec<String>,
}

impl RobotState {
    pub fn default() -> Self {
        RobotState {
            mode: RobotMode::Initializing,
            system_status: SystemStatus::Nominal,
            current_task_id: None,
            pose: None,
            battery_state: None,
            nav_status: Some(NavStatus::Idle),
            active_errors: Vec::new(),
        }
    }

    pub fn add_error(&mut self, error: BrainError) {
        let error_string = error.to_string();
        if self.active_errors.len() >= 10 {
            self.active_errors.remove(0);
        }
        self.active_errors.push(error_string);
    }

    pub fn clear_errors(&mut self) {
        self.active_errors.clear();
    }
}

pub struct StateManager {
    state: RobotState,
}

impl StateManager {
    pub fn new() -> Self {
        Self {
            state: RobotState::default(),
        }
    }

    pub fn get_state(&self) -> RobotState {
        self.state.clone()
    }


    /// Acquires a write lock and allows modifying the state via a closure.
    pub fn update_state<F>(&self, updater: F)
    where
        F: FnOnce(&mut RobotState),
    {
        match self.state.write() {
            Ok(mut state_guard) => updater(&mut *state_guard),
            Err(poisoned) => {
                eprintln!("FATAL: State RwLock poisoned: {}", poisoned);
            }
        }
    }
}
