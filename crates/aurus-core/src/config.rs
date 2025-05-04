use config::{Config, ConfigError, File, FileFormat};
use tracing::{info, error, info_span};

const DEFAULT_CONFIG_PATH: &str = "config/default.toml";

pub fn load_config() -> Result<Config, ConfigError> {
    info!("Attempting to load configuration from {}", DEFAULT_CONFIG_PATH);

    let settings = Config::builder()
        .add_source(File::new(DEFAULT_CONFIG_PATH, FileFormat::Toml).required(true))
        .build();

    match settings {
        Ok(config) => {
            info!("Successfully loaded configuration: {:?}", config);
            Ok(config)
        }
        Err(e) => {
            error!("Failed to load configuration: {}", e);
            Err(e)
        }
    }
}
