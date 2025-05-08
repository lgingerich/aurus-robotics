use macroquad::prelude::*;
use std::sync::Arc;
use tokio::sync::broadcast;
use tracing::{warn, error, info};

use crate::blackboard::Pose; // Assuming Pose is pub or in lib.rs

// Function to configure the macroquad window
pub fn window_conf() -> Conf {
    Conf {
        window_title: "Aurus Core Visualization".to_string(),
        window_width: 800,
        window_height: 600,
        high_dpi: true,
        ..
        Default::default()
    }
}

// Target for visualization, should match the one in navigation/compute_twist if it's static
const VIS_TARGET_X: f32 = 5.0;
const VIS_TARGET_Y: f32 = 2.0;
const VIS_SCALE: f32 = 50.0; // pixels per meter

pub async fn run_visualization_loop(mut pose_rx: broadcast::Receiver<Arc<Pose>>) {
    let mut current_pose_for_vis = Pose::default();
    let mut first_pose_received = false;

    info!("Visualization loop starting inside graphics module...");

    loop {
        match pose_rx.try_recv() {
            Ok(pose_arc) => {
                current_pose_for_vis = (*pose_arc).clone();
                if !first_pose_received {
                    first_pose_received = true;
                    // info!(pose = ?current_pose_for_vis, "Visualization received first pose."); // Optional: can be noisy
                }
            }
            Err(broadcast::error::TryRecvError::Empty) => {
                // No new pose, just continue with the last known one
            }
            Err(broadcast::error::TryRecvError::Lagged(_)) => {
                warn!("Visualization pose receiver lagged.");
                 // Attempt to clear the lag by receiving until empty or an error other than Lagged occurs
                while let Err(broadcast::error::TryRecvError::Lagged(_)) = pose_rx.try_recv() {}
                // Try one more time to get a fresh pose
                if let Ok(pose_arc) = pose_rx.try_recv() {
                    current_pose_for_vis = (*pose_arc).clone();
                }
            }
            Err(broadcast::error::TryRecvError::Closed) => {
                error!("Visualization pose channel closed. Exiting visualization loop.");
                break;
            }
        }

        clear_background(LIGHTGRAY);
        let screen_center_x = screen_width() / 2.0;
        let screen_center_y = screen_height() / 2.0;

        draw_circle(
            screen_center_x + VIS_TARGET_X * VIS_SCALE,
            screen_center_y - VIS_TARGET_Y * VIS_SCALE, 
            10.0, 
            RED,
        );

        if first_pose_received {
            let robot_x_screen = screen_center_x + current_pose_for_vis.x as f32 * VIS_SCALE;
            let robot_y_screen = screen_center_y - current_pose_for_vis.y as f32 * VIS_SCALE;
            let robot_th = -current_pose_for_vis.th as f32; 

            let r_size = 15.0;
            let p1 = Vec2::new(
                robot_x_screen + r_size * robot_th.cos(),
                robot_y_screen + r_size * robot_th.sin(),
            );
            let p2 = Vec2::new(
                robot_x_screen + r_size * (robot_th + 2.0 * std::f32::consts::PI / 3.0).cos(),
                robot_y_screen + r_size * (robot_th + 2.0 * std::f32::consts::PI / 3.0).sin(),
            );
            let p3 = Vec2::new(
                robot_x_screen + r_size * (robot_th - 2.0 * std::f32::consts::PI / 3.0).cos(),
                robot_y_screen + r_size * (robot_th - 2.0 * std::f32::consts::PI / 3.0).sin(),
            );
            draw_triangle(p1, p2, p3, BLUE);
            draw_line(robot_x_screen, robot_y_screen, p1.x, p1.y, 2.0, DARKBLUE);
        }
        
        draw_text(&format!("Robot: x={:.2} y={:.2} th={:.2}", current_pose_for_vis.x, current_pose_for_vis.y, current_pose_for_vis.th), 10.0, 20.0, 20.0, BLACK);
        draw_text(&format!("Target: x={:.1} y={:.1}", VIS_TARGET_X, VIS_TARGET_Y), 10.0, 40.0, 20.0, BLACK);

        next_frame().await
    }
} 