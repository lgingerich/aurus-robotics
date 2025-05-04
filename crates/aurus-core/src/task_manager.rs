use crate::state::Pose;

use std::collections::VecDeque;
use tracing::info;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub enum TaskResult {
    Success,
    Cancelled,
    Failure(String),
}

#[derive(Debug, Clone)]
pub enum Task {
    GoTo { pose: Pose, id: Uuid },
    Charge { id: Uuid },
    // Add other task types here
}

impl Task {
    pub fn id(&self) -> Uuid {
        match self {
            Task::GoTo { id, .. } => *id,
            Task::Charge { id } => *id,
        }
    }
}

pub struct TaskManager {
    task_queue: VecDeque<Task>,
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            task_queue: VecDeque::new(),
        }
    }

    /// Adds a task to the back of the queue.
    pub fn add_task(&mut self, task: Task) {
        self.task_queue.push_back(task);
    }

    /// Retrieves and removes the next task from the front of the queue.
    pub fn get_next_task(&mut self) -> Option<Task> {
        match self.task_queue.pop_front() {
            Some(task) => {
                info!("Task retrieved from queue: {:?}", task.id());
                Some(task)
            }
            None => {
                info!("No task in queue");
                None
            }
        }
    }

    /// Peeks at the next task in the queue without removing it.
    pub fn peek_next_task(&self) -> Option<&Task> {
        match self.task_queue.front() {
            Some(task) => {
                info!("Next task in queue: {:?}", task.id());
                Some(task)
            }
            None => {
                info!("No task in queue");
                None
            }
        }
    }

    /// Clears the task queue.
    pub fn clear_queue(&mut self) {
        self.task_queue.clear();
    }
} 