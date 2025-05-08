// use crate::error::NavError;
// use crate::types::{Task, TaskResult};
// use crate::types::{Pose, RobotState};

// use tokio::sync::broadcast;
// use uuid::Uuid;

// #[derive(Clone, Debug)]
// pub enum Event {
//     PoseUpdate { pose: Pose },
//     BatteryStatus { level: f32, charging: bool },
//     NavigationGoalReached { goal_id: Uuid },
//     NavigationFailed { goal_id: Uuid, error: NavError },
//     TaskCompleted { task_id: Uuid, result: TaskResult },
//     TaskFailed { task_id: Uuid, error: String },
//     ComponentError { component_name: String, error: String },
//     Shutdown,
//     CancelTask { task_id: Uuid },
//     ManualCommand { command: ManualCommandType },
//     TaskRequest { task: Task },
//     StateUpdate { state: RobotState },
// }



// pub struct EventBus {
//     pub bus: broadcast::Sender<Event>,
// }

// // NOTE: Broadcast channel is MPMC (Multiple Producer, Multiple Consumer)

// impl EventBus {
//     /// Creates a new EventBus with a specified channel capacity.
//     pub fn new_with_capacity(capacity: usize) -> (Self, broadcast::Receiver<Event>) {
//         let (tx, rx) = broadcast::channel(capacity);
//         (Self { bus: tx }, rx)
//     }

//     /// Creates a new EventBus with a default channel capacity (e.g., 100).
//     pub fn new() -> (Self, broadcast::Receiver<Event>) {
//         Self::new_with_capacity(100) // Default capacity
//     }

//     pub fn subscribe(&self) -> broadcast::Receiver<Event> {
//         self.bus.subscribe()
//     }

//     pub fn publish(&self, event: Event) -> Result<usize, broadcast::error::SendError<Event>> {
//         self.bus.send(event)
//         // Note: broadcast::send returns Ok(number_of_receivers) or Err if no receivers exist.
//         // You might want to handle the Err case depending on your application logic.
//         // For example, logging a warning if no receivers are active.
//     }
// }



use std::sync::Arc;
use tokio::sync::broadcast;

/// Broadcast topic with bounded capacity.
/// `T` must be `Send + Sync` because we hop across threads.
#[derive(Debug, Clone)]
pub struct Topic<T> {
    tx: broadcast::Sender<Arc<T>>,
}

impl<T: Send + Sync + 'static> Topic<T> {
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self { tx }
    }

    pub fn publish(&self, msg: T) {
        let _ = self.tx.send(Arc::new(msg));
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Arc<T>> {
        self.tx.subscribe()
    }
}
