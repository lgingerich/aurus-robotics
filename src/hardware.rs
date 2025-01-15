// Temporary disable warnings for development
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![allow(unused_assignments)]

use std::sync::{Arc, Mutex};

/// Represents the state of various hardware components
#[derive(Debug, Clone)]
pub struct HardwareState {
    // Motors
    pub motor_speeds: Vec<f32>,          // Speed of each motor (-1.0 to 1.0)
    pub encoder_positions: Vec<f32>,      // Encoder positions in radians
    
    // Sensors
    pub imu_orientation: (f32, f32, f32), // Roll, pitch, yaw in radians
    pub imu_acceleration: (f32, f32, f32), // Acceleration in m/s^2
    pub distance_sensors: Vec<f32>,       // Distance readings in meters
    
    // Battery
    pub battery_voltage: f32,             // Battery voltage in volts
    pub battery_current: f32,             // Current draw in amperes
}

/// Mock hardware interface for robotics platform
pub struct MockHardware {
    state: Arc<Mutex<HardwareState>>,
}

impl MockHardware {
    pub fn new(num_motors: usize, num_distance_sensors: usize) -> Self {
        let state = HardwareState {
            motor_speeds: vec![0.0; num_motors],
            encoder_positions: vec![0.0; num_motors],
            imu_orientation: (0.0, 0.0, 0.0),
            imu_acceleration: (0.0, 0.0, 0.0),
            distance_sensors: vec![0.0; num_distance_sensors],
            battery_voltage: 12.0,
            battery_current: 0.0,
        };

        MockHardware {
            state: Arc::new(Mutex::new(state)),
        }
    }

    /// Set the speed for a specific motor
    pub fn set_motor_speed(&mut self, motor_id: usize, speed: f32) -> Result<(), String> {
        let mut state = self.state.lock().unwrap();
        if motor_id >= state.motor_speeds.len() {
            return Err("Motor ID out of range".to_string());
        }
        
        // Clamp speed between -1.0 and 1.0
        let clamped_speed = speed.clamp(-1.0, 1.0);
        state.motor_speeds[motor_id] = clamped_speed;
        
        // Simulate encoder movement based on motor speed
        state.encoder_positions[motor_id] += clamped_speed * 0.1;
        Ok(())
    }

    /// Read the current position of an encoder
    pub fn read_encoder(&self, encoder_id: usize) -> Result<f32, String> {
        let state = self.state.lock().unwrap();
        if encoder_id >= state.encoder_positions.len() {
            return Err("Encoder ID out of range".to_string());
        }
        Ok(state.encoder_positions[encoder_id])
    }

    /// Read IMU orientation
    pub fn read_imu_orientation(&self) -> (f32, f32, f32) {
        let state = self.state.lock().unwrap();
        state.imu_orientation
    }

    /// Read distance sensor
    pub fn read_distance_sensor(&self, sensor_id: usize) -> Result<f32, String> {
        let state = self.state.lock().unwrap();
        if sensor_id >= state.distance_sensors.len() {
            return Err("Distance sensor ID out of range".to_string());
        }
        Ok(state.distance_sensors[sensor_id])
    }

    /// Read battery status
    pub fn read_battery_status(&self) -> (f32, f32) {
        let state = self.state.lock().unwrap();
        (state.battery_voltage, state.battery_current)
    }

    /// Update mock sensor values (useful for testing)
    pub fn update_mock_sensors(&mut self, 
        orientation: Option<(f32, f32, f32)>,
        acceleration: Option<(f32, f32, f32)>,
        distances: Option<Vec<f32>>) {
        let mut state = self.state.lock().unwrap();
        
        if let Some(ori) = orientation {
            state.imu_orientation = ori;
        }
        if let Some(acc) = acceleration {
            state.imu_acceleration = acc;
        }
        if let Some(dist) = distances {
            if dist.len() == state.distance_sensors.len() {
                state.distance_sensors = dist;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_control() {
        let mut hardware = MockHardware::new(4, 2);
        
        // Test setting valid motor speed
        assert!(hardware.set_motor_speed(0, 0.5).is_ok());
        
        // Test motor speed clamping
        assert!(hardware.set_motor_speed(1, 1.5).is_ok());
        let state = hardware.state.lock().unwrap();
        assert_eq!(state.motor_speeds[1], 1.0);
    }

    #[test]
    fn test_sensor_reading() {
        let mut hardware = MockHardware::new(4, 2);
        
        // Update mock sensors
        hardware.update_mock_sensors(
            Some((0.1, 0.2, 0.3)),
            Some((0.0, 0.0, 9.81)),
            Some(vec![0.5, 1.0])
        );

        // Test reading sensors
        let orientation = hardware.read_imu_orientation();
        assert_eq!(orientation, (0.1, 0.2, 0.3));

        let distance = hardware.read_distance_sensor(0).unwrap();
        assert_eq!(distance, 0.5);
    }
}