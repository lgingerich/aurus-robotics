mod blackboard;
mod bus;
mod graphics;
mod motion_control;
mod state_estimation;

use blackboard::{Blackboard, raise_fault, snapshot, touch_cmd};
use bus::Topic;
use graphics::window_conf;
use macroquad::prelude::*;
use spin_sleep::SpinSleeper;
use std::{
    sync::{Arc, RwLock},
    time::{Duration, Instant},
};
use tracing::{error, info, warn};
use tracing_subscriber::{self, EnvFilter};

use aurus_kinematics::{Pose, Twist};
use aurus_navigation;

#[macroquad::main(window_conf)]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive(tracing::Level::INFO.into()))
        .init();

    info!("Aurus Core started. Setting up Tokio runtime and spawning simulation...");

    let tokio_rt = tokio::runtime::Runtime::new().unwrap();

    let pose_topic_main: Topic<Pose> = Topic::new(16);
    let pose_rx_for_vis = pose_topic_main.subscribe();

    let simulation_pose_topic = pose_topic_main.clone();

    tokio_rt.spawn(async move {
        info!("Simulation task started.");
        match run(simulation_pose_topic).await {
            Ok(_) => info!("Simulation tasks finished successfully."),
            Err(e) => {
                error!(
                    "Simulation tasks failed: {:?}. Core simulation might have stopped.",
                    e
                );
            }
        }
    });

    graphics::run_visualization_loop(pose_rx_for_vis).await;
}

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
    std::thread::Builder::new().name("sensor".into()).spawn({
        let bb_clone = Arc::clone(&bb);
        let last_applied_twist_reader_clone = Arc::clone(&last_applied_twist_shared);
        let pose_topic = pose_topic_for_sensor;
        move || {
            info!("Sensor thread started.");
            let sleeper = SpinSleeper::new(10_000);
            let mut current_pose = Pose::default();
            let dt = 0.01;
            loop {
                let applied_twist = last_applied_twist_reader_clone.read().unwrap();
                current_pose =
                    state_estimation::update_pose_from_kinematics(&current_pose, &applied_twist, dt);
                pose_topic.publish(current_pose);
                bb_clone.write().pose = current_pose;
                sleeper.sleep(Duration::from_micros(10_000));
            }
        }
    })?;

    info!("Spawning control thread...");
    std::thread::Builder::new().name("control".into()).spawn({
        let bb_clone = Arc::clone(&bb);
        let mut twist_rx = twist_topic_for_control.subscribe();
        let last_applied_twist_writer_clone = Arc::clone(&last_applied_twist_shared);
        move || {
            info!("Control thread started.");
            let sleeper = SpinSleeper::new(1_000); // 10 ms = 100 Hz
            loop {
                if let Ok(twist) = twist_rx.try_recv() {
                    motion_control::apply_twist(&twist);
                    *last_applied_twist_writer_clone.write().unwrap() = *twist;
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

async fn async_runtime(
    bb: Blackboard,
    mut pose_rx: tokio::sync::broadcast::Receiver<Arc<Pose>>,
    twist_tx_topic: Topic<Twist>,
) -> anyhow::Result<()> {
    info!("Async runtime started.");

    let initial_pose = snapshot(&bb).pose;
    let twist_tx_sender = twist_tx_topic.get_sender();

    tokio::try_join!(
        aurus_navigation::run_nav_task(initial_pose, &mut pose_rx, twist_tx_sender),
        watchdog(bb),
    )?;
    info!("Async runtime finished.");
    Ok(())
}

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

fn trigger_estop() {
    info!("E-stop triggered.");
    std::process::exit(0);
}
