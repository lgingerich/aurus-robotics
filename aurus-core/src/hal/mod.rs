#[cfg(feature = "stm32")]
pub mod stm32;

// #[cfg(feature = "esp32")]
// pub mod esp32;

// #[cfg(feature = "rpi")]
// pub mod rpi;

use embassy_stm32::Peripherals;

// Initialize hardware based on target platform
pub fn init() -> Peripherals {
    #[cfg(feature = "stm32")]
    return stm32::init_peripherals();

    // #[cfg(feature = "esp32")]
    // return esp32::init_peripherals();

    // #[cfg(feature = "rpi")]
    // return rpi::init_peripherals();
}
