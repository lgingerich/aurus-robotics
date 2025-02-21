#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use stm32::MotorDriver;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut motor = MotorDriver::new(p.PA0, p.PA1);
    
    loop {
        info!("Motor on");
        motor.motor_on();
        Timer::after_millis(3000).await;

        info!("Motor off");
        motor.motor_off();
        Timer::after_millis(3000).await;
    }
}