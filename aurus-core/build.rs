use std::fs;
use serde::Deserialize;

#[derive(Deserialize)]
struct Config {
    hardware: Hardware,
}

#[derive(Deserialize)]
struct Hardware {
    chip: String,
    led_pin: String,
}

fn main() {
    println!("cargo:rustc-check-cfg=cfg(led_pin,values(\"PC13\"))");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    // println!("cargo:rustc-link-arg-bins=-Tmemory.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Parse board.toml from project root
    let config_str = fs::read_to_string("../board.toml").expect("Failed to read board.toml");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse board.toml");

    // Set the configurations
    println!("cargo:rustc-cfg=chip=\"{}\"", config.hardware.chip);
    println!("cargo:rustc-cfg=led_pin=\"{}\"", config.hardware.led_pin);
    println!("cargo:rerun-if-changed=../board.toml");
}