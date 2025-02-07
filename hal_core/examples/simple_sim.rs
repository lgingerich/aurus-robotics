use hal_core::{DigitalPin, Pwm, SimPin, SimPwm};
use std::{thread, time::Duration};

fn main() -> anyhow::Result<()> {
    // Create our simulated devices
    let mut digital_pin = SimPin::new();
    let mut pwm = SimPwm::new();

    // Set initial PWM configuration
    pwm.validate_frequency(1000.0, 0.0, 5000.0)?;
    pwm.set_frequency(1000.0)?; // 1kHz
    pwm.validate_duty_cycle(0.0, 0.0, 1.0)?;
    pwm.set_duty_cycle(0.0)?;   // Start at 0%
    
    // Simple simulation loop
    for i in 0..5 {
        println!("\nIteration {}", i);
        
        // Toggle digital pin
        if i % 2 == 0 {
            digital_pin.set_high()?;
        } else {
            digital_pin.set_low()?;
        }
        
        // Gradually increase PWM duty cycle
        let new_duty = (i as f32) * 0.25; // 0%, 25%, 50%, 75%, 100%
        pwm.validate_duty_cycle(new_duty, 0.0, 1.0)?;
        pwm.set_duty_cycle(new_duty)?;
        
        // Print states
        println!("Digital pin state: {}", if digital_pin.is_high()? { "HIGH" } else { "LOW" });
        println!("PWM frequency: {:.1} Hz", pwm.get_frequency()?);
        println!("PWM duty cycle: {:.1}%", pwm.get_duty_cycle()? * 100.0);
        
        // Wait a second
        thread::sleep(Duration::from_secs(1));
    }
    
    Ok(())
}