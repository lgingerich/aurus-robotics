
TODO: 
    - integrate the encoder from the motor
    - add example for two motors
    - what goes above motor abstraction layer?
    - integrate raspberry pi
    - convert from AnyPin to actual Pin type


           USER
            |
            V

            |
            V

            |
            V

            |
            V
Motor Abstraction Layer 
            |
            V
Hardware Abstraction Layer (HAL)




# Motor Abstraction Layer
    ```rust
    // Core motor traits
    pub trait Motor {
        type Error;
        fn initialize(&mut self) -> Result<(), Self::Error>;
        fn stop(&mut self) -> Result<(), Self::Error>;
    }

    pub trait SpeedControl: Motor {
        fn set_speed(&mut self, speed: f32) -> Result<(), Self::Error>;
        fn get_speed(&self) -> Result<f32, Self::Error>;
    }

    // Example: Create and use a specific motor
    let pwm = p.TIM2.into_pwm();
    let dir_pin = p.PA0.into_output();
    let config = PwmDcConfig { pwm_frequency: 20_000, min_duty: 0.1, max_duty: 0.95 };

    let mut motor = PwmDcMotor::new(pwm, dir_pin, config);
    motor.initialize().unwrap();
    motor.set_speed(0.5).unwrap(); // 50% speed
    ```


# Hardware Abstraction Layer
    ```rust
    // Platform-specific pin implementations
    pub trait Pin {
        fn set_high(&mut self);
        fn set_low(&mut self);
    }

    // STM32 implementation
    #[cfg(feature = "stm32")]
    impl<'a, T: StmPin> Pin for Output<'a, T> {
        fn set_high(&mut self) {
            Output::set_high(self)
        }
        fn set_low(&mut self) {
            Output::set_low(self)
        }
    }

    // Example: Get hardware peripherals
    let p = hal::init();
    let pin = p.PA0.into_output();
    ```