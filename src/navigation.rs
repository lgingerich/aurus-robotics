use std::sync::Arc;
use std::time::Duration;

use tokio::sync::broadcast;
use tokio::time;
use tracing;

use crate::blackboard::{Blackboard, snapshot};
use crate::bus::Topic;
use super::{Pose, Twist}; // Pose and Twist are in the parent module (main.rs)

/// 20 Hz navigation task
pub async fn nav_task(
    bb: Blackboard,
    pose_rx: &mut broadcast::Receiver<Arc<Pose>>,
    twist_tx: Topic<Twist>,
) -> anyhow::Result<()> {
    tracing::info!("Navigation task started.");
    let mut ticker = time::interval(Duration::from_millis(50)); // 20 Hz
    // Initialize with the current pose from the blackboard
    let mut current_pose_arc: Arc<Pose> = Arc::new(snapshot(&bb).pose);
    tracing::info!(initial_pose = ?current_pose_arc, "Navigation task initialized with pose");

    loop {
        tokio::select! {
            _ = ticker.tick() => {
                // Use the locally stored current_pose_arc
                let twist = compute_twist(&current_pose_arc);
                tracing::debug!(vx = twist.vx, wz = twist.wz, current_x = current_pose_arc.x, current_y = current_pose_arc.y, current_th = current_pose_arc.th, "Computed twist for navigation");
                twist_tx.publish(twist);
            }
            Ok(new_pose_arc) = pose_rx.recv() => {
                // Update the current pose when a new one is received
                current_pose_arc = new_pose_arc;
                tracing::debug!(new_pose = ?current_pose_arc, "New pose received in nav_task");
            }
            // If pose_rx.recv() returns Err (e.g., channel closed), the select! will propagate the error,
            // and since nav_task returns anyhow::Result<()>, the task will terminate.
        }
    }
}

// This function is only used by nav_task within this module, so it doesn't need to be pub.
fn compute_twist(current_pose: &Pose) -> Twist {
    let target_pose = Pose { x: 5.0, y: 2.0, th: std::f64::consts::FRAC_PI_2 }; // Target: x=5, y=2, theta=PI/2 (90 degrees)

    let kp_linear = 0.5;    // Proportional gain for linear velocity
    let kp_angular = 1.0;   // Proportional gain for angular velocity
    let kp_orientation = 0.8; // Proportional gain for final orientation alignment

    let max_vx = 0.5;       // Max linear velocity (m/s)
    let max_wz = 1.0;       // Max angular velocity (rad/s)

    let distance_threshold = 0.1; // (meters) Stop if closer than this to target position
    let angle_threshold = 0.05;   // (radians) Consider aligned if angle error is less than this

    let dx = target_pose.x - current_pose.x;
    let dy = target_pose.y - current_pose.y;
    let distance = (dx * dx + dy * dy).sqrt();

    if distance < distance_threshold {
        // Close to the target position, focus on final orientation
        let mut dth_final = target_pose.th - current_pose.th;
        // Normalize dth_final to be between -PI and PI
        while dth_final > std::f64::consts::PI {
            dth_final -= 2.0 * std::f64::consts::PI;
        }
        while dth_final < -std::f64::consts::PI {
            dth_final += 2.0 * std::f64::consts::PI;
        }

        if dth_final.abs() < angle_threshold {
            return Twist::default(); // Reached target position and orientation
        } else {
            let mut wz = kp_orientation * dth_final;
            wz = wz.clamp(-max_wz, max_wz);
            return Twist { vx: 0.0, wz };
        }
    }

    let angle_to_goal = dy.atan2(dx);
    let mut dth_nav = angle_to_goal - current_pose.th;

    // Normalize dth_nav to be between -PI and PI
    while dth_nav > std::f64::consts::PI {
        dth_nav -= 2.0 * std::f64::consts::PI;
    }
    while dth_nav < -std::f64::consts::PI {
        dth_nav += 2.0 * std::f64::consts::PI;
    }

    let mut vx = kp_linear * distance;
    let mut wz = kp_angular * dth_nav;

    // Apply saturation
    vx = vx.clamp(0.0, max_vx); // Only move forward towards goal for this simple controller
    wz = wz.clamp(-max_wz, max_wz);

    // If the angle error to reach the goal is too large, prioritize turning
    if dth_nav.abs() > std::f64::consts::FRAC_PI_2 { // Greater than 90 degrees
        vx = 0.0; // Stop moving forward and just turn
    }

    Twist { vx, wz }
} 

pub fn update_pose_from_kinematics(previous_pose: &Pose, applied_twist: &Twist, dt: f64) -> Pose {
    let new_x = previous_pose.x + applied_twist.vx * dt * previous_pose.th.cos();
    let new_y = previous_pose.y + applied_twist.vx * dt * previous_pose.th.sin();
    let mut new_th = previous_pose.th + applied_twist.wz * dt;

    // Normalize theta to be between -PI and PI for consistency
    while new_th > std::f64::consts::PI {
        new_th -= 2.0 * std::f64::consts::PI;
    }
    while new_th < -std::f64::consts::PI {
        new_th += 2.0 * std::f64::consts::PI;
    }
    
    Pose { x: new_x, y: new_y, th: new_th }
}