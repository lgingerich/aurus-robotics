# Implementation Plan: Core Components & Simulation (No RTOS, Simulated Hardware Focus)

This plan details the technical implementation for the Hardware Abstraction Layer (HAL), Core Services, Communication Layer, and Simulation & Testing components, focusing on simulated hardware and avoiding any RTOS dependencies.

## Phase 1: Hardware Abstraction Layer (HAL) with Simulation

**Objective:** Create a flexible HAL and a simulation implementation for rapid prototyping.

**1.1. Target Hardware (Simulated First):**

*   **Initial Focus:**  Fully simulated hardware.  No direct interaction with physical devices in Phase 1.
*   **Future Targets (Represented by Traits):**  The design *will* anticipate eventual support for real hardware:
    *   **Microcontrollers:** STM32F4, ESP32 (represented by traits but not implemented directly).
    *   **SBCs:** Raspberry Pi 4, NVIDIA Jetson Nano (represented by traits).
* **Expansion:** Defined process for adding new *simulated* devices and, later, real hardware.

**1.2. HAL Structure (Rust Crates):**

*   `hal_core`: (Top-level crate)
    *   Defines core traits:  No changes from the previous plan. These remain the central abstraction.  Examples:
        ```rust
        // hal_core/src/lib.rs
        use thiserror::Error;

        #[derive(Error, Debug)]
        pub enum HalError {
            #[error("I2C communication error: {0}")]
            I2cError(String), // Example: String carries underlying error
            #[error("SPI communication error")]
            SpiError,      // Example: Simple error
            // ... other error types
        }

        #[async_trait::async_trait]
        pub trait DigitalInOut {
            async fn set_high(&mut self) -> Result<(), HalError>;
            async fn set_low(&mut self) -> Result<(), HalError>;
            async fn is_high(&self) -> Result<bool, HalError>;
            async fn is_low(&self) -> Result<bool, HalError>;
        }

        #[async_trait::async_trait]
        pub trait Pwm {
            async fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<(), HalError>;
            async fn get_duty_cycle(&self) -> Result<f32, HalError>;
            async fn set_frequency(&mut self, frequency: f32) -> Result<(), HalError>;
            async fn get_frequency(&self) -> Result<f32, HalError>;
        }

        // ... other traits (I2c, Spi, Serial, Adc, Encoder, Timer)
        ```
    *   Re-exports `embedded-hal` traits (even though we won't use them *directly* in Phase 1, this keeps the API consistent).
    *   Defines error types (`thiserror`).
    *   Configuration mechanism (see 1.3).

*   `hal_sim`: (Simulation implementation)
    *   Implements all traits from `hal_core`.  This is the *core* of the simulated hardware.
    *   Uses internal data structures to represent the state of simulated devices.  Examples:
        ```rust
        // hal_sim/src/lib.rs
        use hal_core::{DigitalInOut, HalError, Pwm};
        use std::sync::Arc;
        use tokio::sync::Mutex;

        pub struct SimDigitalPin {
            state: Arc<Mutex<bool>>,
        }

        impl SimDigitalPin {
            pub fn new() -> Self {
                SimDigitalPin {
                    state: Arc::new(Mutex::new(false)),
                }
            }
        }

        #[async_trait::async_trait]
        impl DigitalInOut for SimDigitalPin {
            async fn set_high(&mut self) -> Result<(), HalError> {
                *self.state.lock().await = true;
                Ok(())
            }

            async fn set_low(&mut self) -> Result<(), HalError> {
                *self.state.lock().await = false;
                Ok(())
            }

            async fn is_high(&self) -> Result<bool, HalError> {
                Ok(*self.state.lock().await)
            }

            async fn is_low(&self) -> Result<bool, HalError> {
                Ok(!*self.state.lock().await)
            }
        }

        pub struct SimPwmChannel {
            duty_cycle: Arc<Mutex<f32>>,
            frequency: Arc<Mutex<f32>>,
        }
        
        impl SimPwmChannel {
            pub fn new() -> Self {
                SimPwmChannel {
                    duty_cycle: Arc::new(Mutex::new(0.0)),
                    frequency: Arc::new(Mutex::new(1000.0)), // Default 1kHz
                }
            }
        }

        #[async_trait::async_trait]
        impl Pwm for SimPwmChannel {
            async fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<(), HalError> {
                if duty_cycle >= 0.0 && duty_cycle <= 1.0 {
                    *self.duty_cycle.lock().await = duty_cycle;
                    Ok(())
                } else {
                    Err(HalError::I2cError("Duty cycle out of range".into())) //Example of specific error
                }
            }

            async fn get_duty_cycle(&self) -> Result<f32, HalError> {
                Ok(*self.duty_cycle.lock().await)
            }

            async fn set_frequency(&mut self, frequency: f32) -> Result<(), HalError> {
               *self.frequency.lock().await = frequency;
                Ok(())
            }

            async fn get_frequency(&self) -> Result<f32, HalError> {
               Ok(*self.frequency.lock().await)
            }
        }

        // ... other simulated device implementations
        ```
    *   Provides a factory function to create instances of simulated devices based on the configuration.
    *   Uses `tokio::sync::Mutex` (or `RwLock`) to protect shared mutable state within the simulated devices.
    *   Crucially, `hal_sim` *does not* depend on Bevy or Rapier directly. It's a pure Rust simulation of hardware interfaces.  The *connection* to Bevy/Rapier happens in the `Simulation & Testing` phase.

*   `hal_<platform>` (Empty stubs for now):  These crates *exist* but are empty.  This maintains the structure for future real hardware support.

*   `hal_devices`: (Optional, but recommended)
    *   Higher-level device abstractions (Motor, Servo, Imu, Lidar, Camera).  No changes in principle.  These now work with *either* `hal_sim` *or* (in the future) a real `hal_<platform>`.

**1.3. Implementation Steps (HAL):**

1.  **Environment Setup:** Rust toolchain only. No embedded tools needed yet.
2.  **Create `hal_core`:**  Define traits, errors, and the configuration mechanism.
    *   **Configuration:** Use a `config.toml` file and the `config` crate.  Example:
        ```toml
        # config.toml
        [hal]
        implementation = "sim"  # Crucial: Selects the 'hal_sim' implementation

        [simulated_devices]
            [[simulated_devices.motors]]
            name = "motor1"
            pwm_channel = "pwm_channel_1" # Links to a SimPwmChannel instance
            dir_pin_a = "gpio_pin_1"    # Links to a SimDigitalPin
            dir_pin_b = "gpio_pin_2"

            [[simulated_devices.sensors]]
            name = "imu1"
            type = "simulated_imu"      # Links to a specific simulated sensor type

        # ... other simulated devices
        ```
        ```rust
        // Example of loading configuration
        use config::{Config, File};
        use serde::Deserialize;

        #[derive(Deserialize, Debug)]
        pub struct HalSettings {
            pub implementation: String,
        }

        #[derive(Deserialize, Debug)]
        pub struct SimulatedDevices {
            pub motors: Option<Vec<SimulatedMotorConfig>>,
            pub sensors: Option<Vec<SimulatedSensorConfig>>
        }

        #[derive(Deserialize, Debug)]
        pub struct SimulatedMotorConfig{
            pub name: String,
            pub pwm_channel: String,
            pub dir_pin_a: String,
            pub dir_pin_b: String,
        }

        #[derive(Deserialize, Debug)]
        pub struct SimulatedSensorConfig{
            pub name: String,
            pub r#type: String, //Using raw identifier, because `type` is reserved keyword
        }

        #[derive(Deserialize, Debug)]
        pub struct Settings {
            pub hal: HalSettings,
            pub simulated_devices: Option<SimulatedDevices>
        }

        pub fn load_config() -> Result<Settings, config::ConfigError> {
            let settings = Config::builder()
                .add_source(File::with_name("config.toml"))
                .build()?;

            settings.try_deserialize()
        }


        //In main or initialization code:

        // let config = hal_core::load_config().expect("Failed to load config");

        // if config.hal.implementation == "sim" {
        //     // Initialize hal_sim based on config.simulated_devices
        //     // ...
        // } else {
        //     // ... handle other HAL implementations (future)
        // }
        ```
3.  **Create `hal_sim`:**  Implement *all* `hal_core` traits with simulated behavior.  This is the most substantial part of Phase 1.
4.  **Create Empty `hal_<platform>` Crates:**  Just create the crates; no implementation needed yet.
5.  **Implement `hal_devices` (Iterative):**  These abstractions will now work with `hal_sim`.
6.  **Thorough Testing:**  Write unit tests for `hal_core` (trait definitions) and *extensive* unit and integration tests for `hal_sim`.  Test all simulated device behaviors.
7.  **Documentation:**  Use `cargo doc`.

**1.4. Asynchronous Considerations:** Unchanged. Use `tokio` and `async-trait`.

**1.5. Error Handling:** Unchanged. Use `thiserror` and `Result`.

## Phase 2: Core Services

**Objective:** Implement core services, interacting with the HAL (now `hal_sim`).

**2.1. Core Services Structure:**  No significant changes from the previous plan.  The key difference is that the Core Services will now *always* use the HAL traits, and in Phase 1, these traits will be implemented by `hal_sim`.

*   **`state_manager`:**  Manages the robot's state.  No changes.
*   **`control_manager`:**  Implements control loops.  No changes.  Example PID controller:
    ```rust
    // control_manager/src/pid.rs
    use crate::{ControlLoop, ControlOutput, ControlError};
    use hal_core::{HalError}; // Or specific error type, like PwmError if applicable.
    use crate::state_manager::State;
    #[async_trait::async_trait]
    impl ControlLoop for PidController {
        async fn execute(&mut self, state: &State) -> Result<ControlOutput, ControlError> {
          //Simple example
          let error = self.setpoint - state.position;
          let output = self.kp * error;
          Ok(ControlOutput{motor_speed: output})
        }
    }

    pub struct PidController {
        kp: f32,
        ki: f32, //Not used in example
        kd: f32, //Not used in example
        setpoint: f32
    }
    #[derive(Debug)]
    pub struct ControlOutput {
        pub motor_speed: f32,
    }
    ```
*   **`safety_monitor`:**  Implements safety rules.  No changes.
*   **`config_manager`:** Loads configuration.  No changes.
*   **`logger` & `diagnostics`:**  Logs and monitors system health.  No changes.

**2.2. Implementation Steps (Core Services):**  No significant changes.  The Core Services are designed to be independent of the specific HAL implementation.

## Phase 3: Communication Layer

**Objective:** Implement communication using Zenoh.

**3.1. Communication Structure:**  No significant changes from the previous plan. The Core Services will publish and subscribe to Zenoh topics, and `hal_sim` will now also interact with Zenoh.

*   **Zenoh Integration:**
    *   Use the `zenoh` crate.
    *   Define Zenoh key expressions (topics).
    *   Implement publishers and subscribers in Core Services *and* `hal_sim`.  This is crucial: `hal_sim` publishes simulated sensor data and subscribes to actuator commands.
*   **Message Serialization:**  Use `postcard`, `bincode`, or `protobuf`.
* **Message Router (Within Comms Layer):** Unchanged.

**3.2. Implementation Steps (Communication Layer):** No significant changes. The important addition is that `hal_sim` now actively participates in Zenoh communication.

## Phase 4: Simulation & Testing (Bevy/Rapier Integration)

**Objective:** Create a simulation environment and connect it to `hal_sim`.

**4.1. Simulation Structure:**

*   **Bevy Engine Integration:**
    *   Use Bevy for the simulation loop and visualization.
    *   Create Bevy entities and components.

*   **Rapier Physics Integration:**
    *   Use Rapier for physics.
    *   Create rigid bodies, colliders, and joints.
    *   Integrate with Bevy using `bevy_rapier`.

*   **Simulated Sensors (in Bevy):**
    *   Implement simulated sensors *within* Bevy.  These sensors generate data based on the Bevy/Rapier simulation. Example (simplified IMU):
        ```rust
        // simulation/src/sensors.rs
        use bevy::prelude::*;
        use bevy_rapier3d::prelude::*; // Or bevy_rapier2d
        use zenoh::prelude::sync::*;     //For publishing to Zenoh

        #[derive(Component)]
        pub struct SimulatedImu {
            pub publisher: zenoh::publication::Publisher<'static>, // Zenoh publisher
        }

        pub fn imu_system(
            mut query: Query<(&mut SimulatedImu, &Transform, &Velocity)>,
            context: Res<ZenohContext>, // Assuming you have a Zenoh context resource
        ) {
            for (mut imu, transform, velocity) in query.iter_mut() {
                // Simulate IMU readings based on transform and velocity
                let acceleration = velocity.linvel; // Simplified example
                let angular_velocity = velocity.angvel;

                // Create an IMU data message (using your chosen serialization)
                let imu_data = ImuData {
                    acceleration: (acceleration.x, acceleration.y, acceleration.z),
                    angular_velocity: (angular_velocity.x, angular_velocity.y, angular_velocity.z),
                };
                let encoded_data = postcard::to_allocvec(&imu_data).unwrap(); // Example using postcard

                // Publish the data to Zenoh
                 imu.publisher.put(encoded_data).res().unwrap();
            }
        }

        #[derive(serde::Serialize, serde::Deserialize)]
        struct ImuData {
            acceleration: (f32, f32, f32),
            angular_velocity: (f32, f32, f32),
        }
        // Add Bevy resource for initializing Zenoh:
        #[derive(Resource)]
        struct ZenohContext {
            context: Context,
        }
        ```

    *   Publish simulated sensor data to Zenoh *using the same topics as real sensors*.

*   **Connecting `hal_sim` to Bevy/Rapier:**
    *   This is the key integration step.  Instead of directly connecting `hal_sim` to Bevy, we use Zenoh as the intermediary.
    *   **Bevy publishes actuator commands to Zenoh.**  For example, a Bevy system might read desired joint velocities from the `control_manager` (via Zenoh) and publish them to the `/robot/commands` topic.
    *   **`hal_sim` subscribes to these actuator commands.** The `hal_sim` crate, running in its own `tokio` task, subscribes to `/robot/commands`.  It receives the commands and updates its internal simulated device states (e.g., sets the `duty_cycle` of a `SimPwmChannel`).
    *   **`hal_sim` publishes simulated sensor data to Zenoh.**  For example, a `SimulatedEncoder` in `hal_sim` might periodically publish its current count to `/sensors/encoder1`.
    *   **Bevy subscribes to this simulated sensor data.** The Bevy `imu_system` (example above) subscribes to `/sensors/imu1`. It receives the simulated IMU data and uses it to update the Bevy/Rapier simulation (e.g., applying forces to the robot).

*   **Deterministic Replay:**  Record Zenoh messages (both commands and sensor data) for deterministic replay.  This simplifies debugging.

*   **CI/CD Integration (Future):** Run simulation tests in CI/CD.

**4.2. Implementation Steps (Simulation & Testing):**

1.  **Set up Bevy and Rapier:**  Create a Bevy project with `bevy_rapier`.
2.  **Create Robot Model:** Define the robot's structure.
3.  **Create Environment:** Create a simple environment.
4.  **Implement Simulated Sensors (in Bevy):**  Create Bevy systems that generate sensor data and publish it to Zenoh.
5.  **Connect `hal_sim` to Zenoh:**
    *   In `hal_sim`, create Zenoh subscribers for actuator commands.
    *   In `hal_sim`, create Zenoh publishers for simulated sensor data.
    *   Ensure `hal_sim` runs in a separate `tokio` task.
6.  **Connect Bevy to Zenoh:**
    *   Create Bevy systems that subscribe to simulated sensor data from Zenoh.
    *   Create Bevy systems that publish actuator commands to Zenoh.
7.  **Testing:** Write tests that use the Bevy/Rapier simulation and verify interactions via Zenoh.
8.  **Deterministic Replay:** Implement recording and replay of Zenoh messages.
9. **CI/CD (future):** Run simulation tests in CI/CD.

## Key Changes and Advantages:

*   **Fully Simulated HAL:**  `hal_sim` provides a complete simulation of hardware, allowing development to proceed without real hardware.
*   **Zenoh as the Integration Layer:**  Zenoh is the *only* communication mechanism between Bevy/Rapier and `hal_sim`. This enforces a clean separation and makes switching to real hardware easier (just replace `hal_sim` with `hal_<platform>`).
*   **No RTOS:**  The entire system runs on a standard desktop operating system using `tokio` for asynchronous operations.
*   **Clear Separation of Concerns:**
    *   `hal_core`: Defines the abstract hardware interfaces.
    *   `hal_sim`: Simulates the hardware.
    *   `hal_devices`: Provides higher-level device abstractions.
    *   Core Services: Implement robot logic.
    *   Communication Layer: Handles communication via Zenoh.
    *   Bevy/Rapier: Provides the visual and physics simulation.
* **More Code Examples:** Added code snippets for clarity, including configuration, trait definitions, and simulated device implementations.
* **Bevy Sensor Integration:** Demonstrated how simulated sensors are implemented *within* Bevy and how they interact with Zenoh.

This revised plan provides a robust and flexible foundation for developing a robotics platform, starting with a fully simulated environment and designed for eventual integration with real hardware. The use of Zenoh as the central communication bus is crucial for achieving this flexibility.