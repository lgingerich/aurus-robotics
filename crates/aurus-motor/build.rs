use serde::Deserialize;
use std::fs;

#[derive(Deserialize)]
struct Config {
    hardware: Hardware,
}

#[derive(Deserialize)]
struct Hardware {
    chip: String,
}

fn main() {
    // Parse board.toml from project root
    let config_str = fs::read_to_string("../../board.toml").expect("Failed to read board.toml");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse board.toml");

    // Set the chip configuration
    println!("cargo:rustc-cfg=chip=\"{}\"", config.hardware.chip);
    println!("cargo:rerun-if-changed=../../board.toml");
}
