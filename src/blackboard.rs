use parking_lot::RwLock;
use std::{sync::Arc, time::Instant};

use aurus_kinematics::{Pose, Twist};

#[derive(Clone)]
pub struct State {
    pub pose: Pose,
    pub twist: Twist,
    pub battery_pct: u8,
    pub last_cmd_ts: Instant,
    pub faults: Vec<String>,
}

impl Default for State {
    fn default() -> Self {
        State {
            pose: Pose::default(),
            twist: Twist::default(),
            battery_pct: 0,
            last_cmd_ts: Instant::now(),
            faults: Vec::new(),
        }
    }
}

pub type Blackboard = Arc<RwLock<State>>;

pub fn snapshot(bb: &Blackboard) -> State {
    (*bb.read()).clone()
}

pub fn touch_cmd(bb: &Blackboard) {
    bb.write().last_cmd_ts = Instant::now();
}

pub fn raise_fault(bb: &Blackboard, msg: &str) {
    let mut g = bb.write();
    if !g.faults.iter().any(|s| s == msg) {
        g.faults.push(msg.to_string());
    }
}
