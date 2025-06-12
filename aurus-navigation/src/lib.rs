pub mod astar;
pub mod error;
pub mod map;
pub mod slam;

use std::{sync::Arc, time::Duration};
use tokio::sync::broadcast;
use tokio::time;
use tracing::{debug, info};

use aurus_kinematics::{Pose, Twist};

fn compute_twist(current_pose: &Pose) -> Twist {
    let target_pose = Pose {
        x: 5.0,
        y: 2.0,
        theta: std::f64::consts::FRAC_PI_2,
    }; // Target: x=5, y=2, theta=PI/2 (90 degrees)

    let kp_linear = 0.5; // Proportional gain for linear velocity
    let kp_angular = 1.0; // Proportional gain for angular velocity
    let kp_orientation = 0.8; // Proportional gain for final orientation alignment

    let max_vx = 0.5; // Max linear velocity (m/s)
    let max_wz = 1.0; // Max angular velocity (rad/s)

    let distance_threshold = 0.1; // (meters) Stop if closer than this to target position
    let angle_threshold = 0.05; // (radians) Consider aligned if angle error is less than this

    let dx = target_pose.x - current_pose.x;
    let dy = target_pose.y - current_pose.y;
    let distance = (dx * dx + dy * dy).sqrt();

    if distance < distance_threshold {
        // Close to the target position, focus on final orientation
        let mut dtheta_final = target_pose.theta - current_pose.theta;
        // Normalize dtheta_final to be between -PI and PI
        while dtheta_final > std::f64::consts::PI {
            dtheta_final -= 2.0 * std::f64::consts::PI;
        }
        while dtheta_final < -std::f64::consts::PI {
            dtheta_final += 2.0 * std::f64::consts::PI;
        }

        if dtheta_final.abs() < angle_threshold {
            return Twist::default(); // Reached target position and orientation
        } else {
            let mut wz = kp_orientation * dtheta_final;
            wz = wz.clamp(-max_wz, max_wz);
            return Twist { vx: 0.0, wz };
        }
    }

    let angle_to_goal = dy.atan2(dx);
    let mut dtheta_nav = angle_to_goal - current_pose.theta;

    // Normalize dtheta_nav to be between -PI and PI
    while dtheta_nav > std::f64::consts::PI {
        dtheta_nav -= 2.0 * std::f64::consts::PI;
    }
    while dtheta_nav < -std::f64::consts::PI {
        dtheta_nav += 2.0 * std::f64::consts::PI;
    }

    let mut vx = kp_linear * distance;
    let mut wz = kp_angular * dtheta_nav;

    // Apply saturation
    vx = vx.clamp(0.0, max_vx); // Only move forward towards goal for this simple controller
    wz = wz.clamp(-max_wz, max_wz);

    // If the angle error to reach the goal is too large, prioritize turning
    if dtheta_nav.abs() > std::f64::consts::FRAC_PI_2 {
        // Greater than 90 degrees
        vx = 0.0; // Stop moving forward and just turn
    }

    Twist { vx, wz }
}

/// 20 Hz navigation task, refactored from the main application.
///
/// # Arguments
/// * `initial_pose` - The starting pose for the navigation task.
/// * `pose_rx` - A Tokio broadcast receiver for `Arc<Pose>` updates.
/// * `twist_tx` - A Tokio broadcast sender to publish computed `Arc<Twist>` commands.
pub async fn run_nav_task(
    initial_pose: Pose,
    pose_rx: &mut broadcast::Receiver<Arc<Pose>>,
    twist_tx: broadcast::Sender<Arc<Twist>>,
) -> anyhow::Result<()> {
    info!("Navigation task started in aurus_navigation crate.");
    let mut ticker = time::interval(Duration::from_millis(50)); // 50 ms = 20 Hz
    let mut current_pose_arc: Arc<Pose> = Arc::new(initial_pose);
    info!(initial_pose = ?current_pose_arc, "Navigation task initialized with pose");

    loop {
        tokio::select! {
            _ = ticker.tick() => {
                // Use the locally stored current_pose_arc
                let twist = compute_twist(&current_pose_arc);
                debug!(vx = twist.vx, wz = twist.wz, current_x = current_pose_arc.x, current_y = current_pose_arc.y, current_theta = current_pose_arc.theta, "Computed twist for navigation");
                // Send Arc<Twist>
                if twist_tx.receiver_count() > 0 {
                    if let Err(e) = twist_tx.send(Arc::new(twist)) {
                        // Log the error, but don't bring down the nav task if sending fails (e.g. no subscribers)
                        // However, for critical systems, this might warrant an error propagation.
                        // For now, we match the previous Topic.publish behavior which also logs and continues.
                        tracing::warn!("Failed to publish twist from nav_task: {}", e);
                    }
                }
            }
            result = pose_rx.recv() => {
                match result {
                    Ok(new_pose_arc) => {
                        // Update the current pose when a new one is received
                        current_pose_arc = new_pose_arc;
                        debug!(new_pose = ?current_pose_arc, "New pose received in nav_task");
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::warn!("Pose receiver lagged by {} messages in nav_task.", n);
                        // Attempt to receive again to clear the lag, or resync.
                        // For simplicity, we'll just continue and try to catch up.
                        // In a real system, you might want to fetch the latest available pose.
                        continue;
                    }
                    Err(broadcast::error::RecvError::Closed) => {
                        tracing::error!("Pose channel closed. Navigation task cannot continue.");
                        return Err(anyhow::anyhow!("Pose channel closed for navigation task"));
                    }
                }
            }
        }
    }
}
