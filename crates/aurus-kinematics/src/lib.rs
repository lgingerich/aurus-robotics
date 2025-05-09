use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64, // Radians
}

impl Pose {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Pose { x, y, theta }
    }

    // Helper function to normalize angle to be within [-PI, PI)
    pub fn normalize_angle(angle: f64) -> f64 {
        let a = angle % (2.0 * PI);
        if a >= PI {
            a - 2.0 * PI
        } else if a < -PI {
            a + 2.0 * PI
        } else {
            a
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpeeds {
    pub omega_l: f64, // Radians per second for left wheel
    pub omega_r: f64, // Radians per second for right wheel
}

impl WheelSpeeds {
    pub fn new(omega_l: f64, omega_r: f64) -> Self {
        WheelSpeeds { omega_l, omega_r }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisSpeeds {
    pub v: f64,     // Linear velocity in m/s
    pub omega: f64, // Angular velocity in rad/s
}

impl ChassisSpeeds {
    pub fn new(v: f64, omega: f64) -> Self {
        ChassisSpeeds { v, omega }
    }
}

pub struct DifferentialDriveKinematics {
    wheel_radius: f64,    // meters
    axle_length: f64,     // meters (distance between wheel centers)
}

impl DifferentialDriveKinematics {
    pub fn new(wheel_radius: f64, axle_length: f64) -> Self {
        if wheel_radius <= 0.0 {
            panic!("Wheel radius must be positive.");
        }
        if axle_length <= 0.0 {
            panic!("Axle length must be positive.");
        }
        DifferentialDriveKinematics {
            wheel_radius,
            axle_length,
        }
    }

    /// Calculates the robot\'s chassis speeds (linear and angular velocity)
    /// from the wheel speeds.
    pub fn forward_kinematics(&self, wheel_speeds: WheelSpeeds) -> ChassisSpeeds {
        let v_l = wheel_speeds.omega_l * self.wheel_radius;
        let v_r = wheel_speeds.omega_r * self.wheel_radius;

        let v = (v_r + v_l) / 2.0;
        let omega = (v_r - v_l) / self.axle_length;

        ChassisSpeeds::new(v, omega)
    }

    /// Updates the robot\'s pose given its current pose, chassis speeds, and time delta.
    pub fn update_pose(&self, current_pose: Pose, chassis_speeds: ChassisSpeeds, dt: f64) -> Pose {
        if dt < 0.0 {
            panic!("Time delta (dt) must be non-negative.");
        }

        let delta_x = chassis_speeds.v * current_pose.theta.cos() * dt;
        let delta_y = chassis_speeds.v * current_pose.theta.sin() * dt;
        let delta_theta = chassis_speeds.omega * dt;

        Pose {
            x: current_pose.x + delta_x,
            y: current_pose.y + delta_y,
            theta: Pose::normalize_angle(current_pose.theta + delta_theta),
        }
    }

    /// Convenience function to update pose directly from wheel speeds and dt.
    pub fn update_pose_from_wheel_speeds(
        &self,
        current_pose: Pose,
        wheel_speeds: WheelSpeeds,
        dt: f64,
    ) -> Pose {
        let chassis_speeds = self.forward_kinematics(wheel_speeds);
        self.update_pose(current_pose, chassis_speeds, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const EPSILON: f64 = 1e-6;

    #[test]
    fn test_pose_normalization() {
        assert!((Pose::normalize_angle(0.0) - 0.0).abs() < EPSILON);
        assert!((Pose::normalize_angle(PI) - PI).abs() < EPSILON); // Upper bound, becomes -PI
        assert!((Pose::normalize_angle(PI - EPSILON) - (PI - EPSILON)).abs() < EPSILON);
        assert!((Pose::normalize_angle(-PI) - -PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(3.0 * PI) - PI).abs() < EPSILON); // Upper bound, becomes -PI
        assert!((Pose::normalize_angle(2.5 * PI) - 0.5 * PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(-2.5 * PI) - -0.5 * PI).abs() < EPSILON);
        assert!((Pose::normalize_angle(-3.0 * PI) - -PI).abs() < EPSILON);
    }
    
    #[test]
    fn test_kinematics_constructor() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        assert_eq!(kinematics.wheel_radius, 0.1);
        assert_eq!(kinematics.axle_length, 0.5);
    }

    #[test]
    #[should_panic]
    fn test_constructor_invalid_radius() {
        DifferentialDriveKinematics::new(0.0, 0.5);
    }

    #[test]
    #[should_panic]
    fn test_constructor_invalid_axle_length() {
        DifferentialDriveKinematics::new(0.1, -0.5);
    }

    #[test]
    fn test_forward_kinematics_straight() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(10.0, 10.0); // Both wheels 10 rad/s
                                                          // v_l = 10 * 0.1 = 1 m/s
                                                          // v_r = 10 * 0.1 = 1 m/s
                                                          // v = (1 + 1) / 2 = 1 m/s
                                                          // omega = (1 - 1) / 0.5 = 0 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 1.0).abs() < EPSILON);
        assert!((chassis_speeds.omega - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_forward_kinematics_pivot_turn() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(-5.0, 5.0);   // Left -5 rad/s, Right 5 rad/s
                                                          // v_l = -5 * 0.1 = -0.5 m/s
                                                          // v_r = 5 * 0.1 = 0.5 m/s
                                                          // v = (0.5 + (-0.5)) / 2 = 0 m/s
                                                          // omega = (0.5 - (-0.5)) / 0.5 = 1 / 0.5 = 2 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 0.0).abs() < EPSILON);
        assert!((chassis_speeds.omega - 2.0).abs() < EPSILON);
    }
    
    #[test]
    fn test_forward_kinematics_gentle_turn() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5); // r=0.1m, L=0.5m
        let wheel_speeds = WheelSpeeds::new(5.0, 10.0);   // Left 5 rad/s, Right 10 rad/s
                                                          // v_l = 5 * 0.1 = 0.5 m/s
                                                          // v_r = 10 * 0.1 = 1.0 m/s
                                                          // v = (1.0 + 0.5) / 2 = 0.75 m/s
                                                          // omega = (1.0 - 0.5) / 0.5 = 0.5 / 0.5 = 1 rad/s
        let chassis_speeds = kinematics.forward_kinematics(wheel_speeds);
        assert!((chassis_speeds.v - 0.75).abs() < EPSILON);
        assert!((chassis_speeds.omega - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_straight_no_rotation() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        let current_pose = Pose::new(0.0, 0.0, 0.0); // Facing along X-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward, 0 rad/s
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 1*cos(0)*1 = 1
        //           y = 0 + 1*sin(0)*1 = 0
        //           theta = 0 + 0*1 = 0
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt);
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_straight_with_initial_rotation() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        let current_pose = Pose::new(1.0, 1.0, PI / 2.0); // At (1,1), facing along Y-axis
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0); // 1 m/s forward, 0 rad/s
        let dt = 2.0; // 2 seconds

        // Expected: x = 1 + 1*cos(PI/2)*2 = 1 + 0 = 1
        //           y = 1 + 1*sin(PI/2)*2 = 1 + 2 = 3
        //           theta = PI/2 + 0*2 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt);
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 3.0).abs() < EPSILON);
        assert!((new_pose.theta - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_pivot_turn_no_translation() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let chassis_speeds = ChassisSpeeds::new(0.0, PI / 2.0); // 0 m/s, PI/2 rad/s (90 deg/s)
        let dt = 1.0; // 1 second

        // Expected: x = 0 + 0*cos(0)*1 = 0
        //           y = 0 + 0*sin(0)*1 = 0
        //           theta = 0 + (PI/2)*1 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt);
        assert!((new_pose.x - 0.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_combined_motion() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        let current_pose = Pose::new(1.0, 2.0, PI / 4.0); // At (1,2), 45 deg
        let chassis_speeds = ChassisSpeeds::new(1.0, PI / 2.0); // 1 m/s, PI/2 rad/s
        let dt = 0.5; // 0.5 seconds

        // Expected intermediate values:
        // delta_x = 1.0 * cos(PI/4) * 0.5 = 1.0 * (sqrt(2)/2) * 0.5 = sqrt(2)/4 approx 0.35355
        // delta_y = 1.0 * sin(PI/4) * 0.5 = 1.0 * (sqrt(2)/2) * 0.5 = sqrt(2)/4 approx 0.35355
        // delta_theta = (PI/2) * 0.5 = PI/4
        //
        // x_new = 1.0 + 0.35355 = 1.35355
        // y_new = 2.0 + 0.35355 = 2.35355
        // theta_new = PI/4 + PI/4 = PI/2
        let new_pose = kinematics.update_pose(current_pose, chassis_speeds, dt);
        let expected_x = 1.0 + (2.0_f64.sqrt() / 4.0);
        let expected_y = 2.0 + (2.0_f64.sqrt() / 4.0);
        let expected_theta = PI / 2.0;

        assert!((new_pose.x - expected_x).abs() < EPSILON);
        assert!((new_pose.y - expected_y).abs() < EPSILON);
        assert!((new_pose.theta - expected_theta).abs() < EPSILON);
    }

    #[test]
    #[should_panic]
    fn test_update_pose_negative_dt() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5);
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let chassis_speeds = ChassisSpeeds::new(1.0, 0.0);
        kinematics.update_pose(current_pose, chassis_speeds, -0.1);
    }
    
    #[test]
    fn test_update_pose_from_wheel_speeds_straight() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5); // r=0.1m, L=0.5m
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let wheel_speeds = WheelSpeeds::new(10.0, 10.0); // Both wheels 10 rad/s => v=1m/s, omega=0rad/s
        let dt = 1.0;

        let new_pose = kinematics.update_pose_from_wheel_speeds(current_pose, wheel_speeds, dt);
        assert!((new_pose.x - 1.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_update_pose_from_wheel_speeds_pivot_turn() {
        let kinematics = DifferentialDriveKinematics::new(0.1, 0.5); // r=0.1m, L=0.5m
        let current_pose = Pose::new(0.0, 0.0, 0.0);
        let wheel_speeds = WheelSpeeds::new(-PI, PI);   // Left -PI rad/s, Right PI rad/s
                                                        // v_l = -PI * 0.1 = -0.1*PI
                                                        // v_r = PI * 0.1 = 0.1*PI
                                                        // v = 0 m/s
                                                        // omega = (0.1*PI - (-0.1*PI)) / 0.5 = (0.2*PI) / 0.5 = 0.4*PI rad/s
        let dt = 1.0; // 1 second
        // Expected theta_new = 0 + 0.4*PI*1 = 0.4*PI
        
        let new_pose = kinematics.update_pose_from_wheel_speeds(current_pose, wheel_speeds, dt);
        assert!((new_pose.x - 0.0).abs() < EPSILON);
        assert!((new_pose.y - 0.0).abs() < EPSILON);
        assert!((new_pose.theta - (0.4 * PI)).abs() < EPSILON);
    }
}
