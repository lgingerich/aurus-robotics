use aurus_core::{
    config::load_config,
    state::StateManager,
    task_manager::TaskManager,
};

use tokio;
use tracing::{info, error};
use tracing_subscriber::{self, EnvFilter};

#[tokio::main]
async fn main() {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive(tracing::Level::INFO.into()))
        .init();

    info!("Starting Aurus Core...");

    // Run the main application logic
    match run().await {
        Ok(_) => info!("Application finished successfully."),
        Err(e) => {
            tracing::error!("Application failed: {:?}", e);
            std::process::exit(1); // Exit with a non-zero code on failure
        }
    }
}

async fn run() -> anyhow::Result<()> {
    // Load config
    let config = load_config()?;

    info!("Starting components...");

    // Initialize core components
    let state_manager = StateManager::new();
    let task_manager = TaskManager::new();

    Ok(())
}
