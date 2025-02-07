use anyhow::Result;

pub trait DigitalPin {
    fn set_high(&mut self) -> Result<()>;
    fn set_low(&mut self) -> Result<()>;
    fn is_high(&self) -> Result<bool>;
}

pub struct SimPin {
    state: bool, // true = high, false = low
}

impl SimPin {
    pub fn new() -> Self {
        SimPin { state: false }
    }
}

impl DigitalPin for SimPin {
    fn set_high(&mut self) -> Result<()> {
        self.state = true;
        Ok(())
    }

    fn set_low(&mut self) -> Result<()> {
        self.state = false;
        Ok(())
    }

    fn is_high(&self) -> Result<bool> {
        Ok(self.state)
    }   
}


pub trait Pwm {
    fn set_frequency(&mut self, frequency: f32) -> Result<()>;
    fn get_frequency(&self) -> Result<f32>;
    fn validate_frequency(&self, frequency: f32, min: f32, max: f32) -> Result<()>;
    fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()>;
    fn get_duty_cycle(&self) -> Result<f32>;
    fn validate_duty_cycle(&self, duty_cycle: f32, min: f32, max: f32) -> Result<()>;
}

pub struct SimPwm {
    frequency: f32,
    duty_cycle: f32,
}

impl SimPwm {
    pub fn new() -> Self {
        SimPwm { frequency: 0.0, duty_cycle: 0.0 }
    }
}

impl Pwm for SimPwm {
    fn set_frequency(&mut self, frequency: f32) -> Result<()> {
        self.frequency = frequency;
        Ok(())
    }

    fn get_frequency(&self) -> Result<f32> {
        Ok(self.frequency)
    }

    fn validate_frequency(&self, frequency: f32, min: f32, max: f32) -> Result<()> {
        if frequency < min || frequency > max {
            return Err(anyhow::anyhow!("Frequency out of range"));
        }
        Ok(())
    }

    fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()> {
        self.duty_cycle = duty_cycle;
        Ok(())
    }

    fn get_duty_cycle(&self) -> Result<f32> {
        Ok(self.duty_cycle)
    }

    fn validate_duty_cycle(&self, duty_cycle: f32, min: f32, max: f32) -> Result<()> {
        if duty_cycle < min || duty_cycle > max {
            return Err(anyhow::anyhow!("Duty cycle out of range"));
        }
        Ok(())
    }
}
