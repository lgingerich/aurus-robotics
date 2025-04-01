#![no_std]
#![no_main]

pub mod motor;

#[cfg(test)]
#[defmt_test::tests]
mod hardware_tests {
    use crate::motor::pwm::duty_cycle_to_percent; // Import the helper function
    use embassy_stm32::gpio::Level;

    // --- Mock Implementations ---

    // Improved Mock Pin that tracks its level
    struct MockPin {
        level: Level,
    }

    impl MockPin {
        fn new(initial_level: Level) -> Self {
            Self {
                level: initial_level,
            }
        }

        fn set_high(&mut self) {
            self.level = Level::High;
        }

        fn set_low(&mut self) {
            self.level = Level::Low;
        }

        fn is_set_high(&self) -> bool {
            self.level == Level::High
        }

        // Optional: Add is_set_low for clarity if needed
        // fn is_set_low(&self) -> bool {
        //     self.level == Level::Low
        // }
    }

    // Mock for PWM channel (mostly unchanged, using a fixed max_duty_cycle for simplicity)
    struct MockPwmChannel {
        enabled: bool,
        duty_cycle: u16,
        max_duty_cycle: u16, // Fixed at 1000 for these tests
    }

    impl MockPwmChannel {
        fn new() -> Self {
            Self {
                enabled: false,
                duty_cycle: 0,
                max_duty_cycle: 1000,
            }
        }

        fn enable(&mut self) {
            self.enabled = true;
        }

        fn disable(&mut self) {
            self.enabled = false;
        }

        fn is_enabled(&self) -> bool {
            self.enabled
        }

        fn set_duty_cycle(&mut self, duty: u16) {
            // Add assertion for safety, mirroring real PWM behavior
            defmt::assert!(duty <= self.max_duty_cycle);
            self.duty_cycle = duty;
        }

        fn current_duty_cycle(&self) -> u16 {
            self.duty_cycle
        }

        fn max_duty_cycle(&self) -> u16 {
            self.max_duty_cycle
        }

        fn set_duty_cycle_percent(&mut self, percent: u8) {
            defmt::assert!(percent <= 100);
            self.duty_cycle = (percent as u32 * self.max_duty_cycle as u32 / 100) as u16;
        }
    }

    // --- Test Functions ---

    #[test]
    fn test_gpio_direction_control() {
        let mut pin = MockPin::new(Level::High);
        // Test forward direction
        pin.set_high();
        defmt::assert!(
            pin.is_set_high(),
            "GPIO Dir Pin: Should be high for forward"
        );
        // Test reverse direction
        pin.set_low();
        defmt::assert!(
            !pin.is_set_high(),
            "GPIO Dir Pin: Should be low for reverse"
        );
    }

    #[test]
    fn test_gpio_enable_control() {
        let mut pin = MockPin::new(Level::Low); // Enable pin starts low (disabled)
                                                // Test enable
        pin.set_high();
        defmt::assert!(
            pin.is_set_high(),
            "GPIO Enable Pin: Should be high when enabled"
        );
        // Test disable
        pin.set_low();
        defmt::assert!(
            !pin.is_set_high(),
            "GPIO Enable Pin: Should be low when disabled"
        );
    }

    #[test]
    fn test_pwm_direction_control() {
        let mut pin = MockPin::new(Level::High);
        // Test forward direction
        pin.set_high();
        defmt::assert!(pin.is_set_high(), "PWM Dir Pin: Should be high for forward");
        // Test reverse direction
        pin.set_low();
        defmt::assert!(!pin.is_set_high(), "PWM Dir Pin: Should be low for reverse");
    }

    #[test]
    fn test_pwm_channel_control() {
        let mut pwm_ch = MockPwmChannel::new();
        // Test initial state
        defmt::assert!(
            !pwm_ch.is_enabled(),
            "PWM Channel: Should be disabled initially"
        );
        defmt::assert_eq!(
            pwm_ch.current_duty_cycle(),
            0,
            "PWM Channel: Initial duty cycle should be 0"
        );
        // Test enable/disable
        pwm_ch.enable();
        defmt::assert!(pwm_ch.is_enabled(), "PWM Channel: Should be enabled");
        pwm_ch.disable();
        defmt::assert!(!pwm_ch.is_enabled(), "PWM Channel: Should be disabled");
        // Test duty cycle control
        pwm_ch.set_duty_cycle(500);
        defmt::assert_eq!(
            pwm_ch.current_duty_cycle(),
            500,
            "PWM Channel: Duty cycle should be set correctly"
        );
        // Test percentage-based control
        pwm_ch.set_duty_cycle_percent(50);
        defmt::assert_eq!(
            pwm_ch.current_duty_cycle(),
            500,
            "PWM Channel: 50% should be 500/1000"
        );
    }

    #[test]
    fn test_duty_cycle_conversion() {
        defmt::assert_eq!(
            duty_cycle_to_percent(500, 1000),
            50,
            "Conversion: 500/1000 should be 50%"
        );
        defmt::assert_eq!(
            duty_cycle_to_percent(0, 1000),
            0,
            "Conversion: 0/1000 should be 0%"
        );
        defmt::assert_eq!(
            duty_cycle_to_percent(1000, 1000),
            100,
            "Conversion: 1000/1000 should be 100%"
        );
    }
}
