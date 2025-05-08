mod blackboard;   // brings `blackboard.rs` in as `crate::blackboard`
mod bus;          // brings `bus.rs` in as `crate::bus`
mod navigation;   // brings `navigation.rs` in as `crate::navigation`
mod graphics;     // brings `graphics.rs` in as `crate::graphics`

use blackboard::{Blackboard, Pose, snapshot, touch_cmd, raise_fault};
use bus::Topic;
use graphics::window_conf; // Import window_conf directly

use macroquad::prelude::*;
use spin_sleep::SpinSleeper;
use std::{sync::Arc, time::{Duration, Instant}};
use std::sync::RwLock;
use tokio;
use tracing::{info, warn, error};
use tracing_subscriber::{self, EnvFilter};

/// Simple velocity command used in examples
#[derive(Clone, Default, Debug)]
struct Twist { vx: f64, wz: f64 }

#[macroquad::main(window_conf)]
async fn main() { 
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive(tracing::Level::INFO.into()))
        .init();

    info!("Aurus Core (Macroquad Frontend) Started. Setting up Tokio runtime and spawning simulation...");

    let tokio_rt = tokio::runtime::Runtime::new().unwrap();

    let pose_topic_main: Topic<Pose> = Topic::new(16); 
    let pose_rx_for_vis = pose_topic_main.subscribe(); 
    
    let simulation_pose_topic = pose_topic_main.clone(); 
    
    tokio_rt.spawn(async move {
        info!("Simulation task started.");
        match run(simulation_pose_topic).await { // Call renamed function
            Ok(_) => info!("Simulation tasks finished successfully."),
            Err(e) => {
                error!("Simulation tasks failed: {:?}. Core simulation might have stopped.", e);
                // Depending on severity, you might want to signal the graphics loop or exit.
            }
        }
    });

    graphics::run_visualization_loop(pose_rx_for_vis).await;
}

// Renamed from run_simulation_core_with_topic
async fn run(pose_topic: Topic<Pose>) -> anyhow::Result<()> { 
    info!("Core simulation setup started using provided pose topic.");
    let bb: Blackboard = Arc::default();
    let twist_topic: Topic<Twist> = Topic::new(4);
    let last_applied_twist_shared: Arc<RwLock<Twist>> = Arc::new(RwLock::new(Twist::default()));

    let pose_topic_for_sensor = pose_topic.clone(); 
    let twist_topic_for_control = twist_topic.clone();
    let twist_topic_for_nav = twist_topic.clone();
    let pose_rx_for_nav = pose_topic.subscribe(); 
    
    info!("Spawning sensor thread...");
    std::thread::Builder::new()
        .name("sensor".into())
        .spawn({
            let bb_clone = Arc::clone(&bb);
            let last_applied_twist_reader_clone = Arc::clone(&last_applied_twist_shared);
            let pose_topic = pose_topic_for_sensor; 
            move || {
                info!("Sensor thread started.");
                let sleeper = SpinSleeper::new(10_000);
                let mut current_pose = Pose::default(); 
                let dt = 0.01; 
                loop {
                    let applied_twist = last_applied_twist_reader_clone.read().unwrap().clone();
                    current_pose = navigation::update_pose_from_kinematics(&current_pose, &applied_twist, dt);
                    pose_topic.publish(current_pose.clone());
                    bb_clone.write().pose = current_pose.clone(); 
                    sleeper.sleep(Duration::from_micros(10_000));
                }
            }
        })?;

    info!("Spawning control thread...");
    std::thread::Builder::new()
        .name("control".into())
        .spawn({
            let bb_clone = Arc::clone(&bb);
            let mut twist_rx = twist_topic_for_control.subscribe();
            let last_applied_twist_writer_clone = Arc::clone(&last_applied_twist_shared);
            move || {
                info!("Control thread started.");
                let sleeper = SpinSleeper::new(1_000);
                loop {
                    if let Ok(twist) = twist_rx.try_recv() {
                        apply_twist(&twist); 
                        *last_applied_twist_writer_clone.write().unwrap() = (*twist).clone();
                        touch_cmd(&bb_clone);
                    }
                    sleeper.sleep(Duration::from_micros(1_000));
                }
            }
        })?;
    
    info!("Starting Tokio runtime for async tasks (navigation, watchdog) within simulation...");
    async_runtime(bb, pose_rx_for_nav, twist_topic_for_nav).await?;

    Ok(())
}

// async fn async_runtime remains the same, but ensure it uses the correct pose_rx from the main pose_topic
async fn async_runtime(
    bb: Blackboard,
    mut pose_rx: tokio::sync::broadcast::Receiver<Arc<Pose>>,
    twist_tx: Topic<Twist>,
) -> anyhow::Result<()> {
    info!("Async runtime started.");
    tokio::try_join!(
        navigation::nav_task(bb.clone(), &mut pose_rx, twist_tx.clone()),
        watchdog(bb),
    )?;
    info!("Async runtime finished.");
    Ok(())
}

// async fn watchdog remains the same
async fn watchdog(bb: Blackboard) -> anyhow::Result<()> {
    info!("Watchdog task started.");
    let mut tick = tokio::time::interval(Duration::from_millis(25));
    loop {
        tick.tick().await;
        let last_cmd_ts = snapshot(&bb).last_cmd_ts;
        let age = Instant::now() - last_cmd_ts;
        if age > Duration::from_millis(100) {
            warn!(?age, last_cmd_ts = ?last_cmd_ts, "Command velocity timeout! Triggering E-stop.");
            trigger_estop();
            raise_fault(&bb, "cmd_vel timeout");
        }
    }
}

// fn apply_twist, trigger_estop remain the same
fn apply_twist(_t: &Twist) {}
fn trigger_estop() {}
