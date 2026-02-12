/// @file aether.hpp
/// @brief Aether: Unified Modular Architecture for Multi-Domain Unmanned Vehicles.
/// @details This module defines the high-level namespace topology for the Aether system,
///          targeting Air, Land, and Sea domains using C++20 and IEEE 1722 TSN.
///          Implements the Dual-Adapter Strategy: Concept (Async Contract) + Local + Remote.
/// @version 0.3
/// @copyright KMX Systems

#include <cstdint>
#include <concepts>
#include <coroutine>

#include <kmx/aether/scheduler.hpp>

/// @namespace aether
/// @brief The root namespace for the unified architecture.
namespace kmx::aether::v0_1
{
    /// @brief Minimal concept for a coroutine-compatible Awaitable (Sender model).
    template<typename T>
    concept awaitable = requires(T t, std::coroutine_handle<> h) {
        { t.await_ready() } -> std::convertible_to<bool>;
        { t.await_suspend(h) };
        { t.await_resume() };
    };

    /// @brief Helper to enforce return types of async operations.
    template<typename T, typename Result>
    concept sender_of = awaitable<T> && requires(T t) {
        { t.await_resume() } -> std::convertible_to<Result>;
    };

    /// @brief Base concept for all Aether services enforcing async behavior.
    template<typename T>
    concept async_service = requires(T t) {
        typename T::service_tag;
    };

    // --- Common Data Types for APIs ---

    struct empty {}; // Void return for senders

    // Generic trivial awaitable for mocking
    template<typename T = void>
    struct task_stub {
        struct promise_type {
            T value;
            task_stub get_return_object() { return {}; }
            std::suspend_never initial_suspend() { return {}; }
            std::suspend_never final_suspend() noexcept { return {}; }
            void return_value(T v) { value = v; }
            void unhandled_exception() {}
        };
        bool await_ready() const { return true; }
        void await_suspend(std::coroutine_handle<>) {}
        T await_resume() { return {}; }
    };

    template<>
    struct task_stub<void> {
        struct promise_type {
            task_stub get_return_object() { return {}; }
            std::suspend_never initial_suspend() { return {}; }
            std::suspend_never final_suspend() noexcept { return {}; }
            void return_void() {}
            void unhandled_exception() {}
        };
        bool await_ready() const { return true; }
        void await_suspend(std::coroutine_handle<>) {}
        void await_resume() {}
    };

    /// @namespace aether::foundation
    /// @brief The bedrock layer providing low-level system abstractions, mathematics, and runtime utilities.
    namespace foundation
    {
        /// @namespace aether::foundation::sys
        /// @brief Operating System primitives and execution models.
        namespace sys
        {
            /// @brief Coroutine executor and task dispatcher (Contract).
            template<typename T> concept scheduler = async_service<T>;

            /// @brief Worker thread management for parallel execution (Contract).
            template<typename T> concept thread_pool = async_service<T>;

            /// @brief Real-time polymorphic memory resources (PMR) (Contract).
            template<typename T> concept memory_pool = async_service<T>;

            /// @brief Hardware independent watchdog timer interface (Contract).
            template<typename T> concept watchdog = async_service<T>;

            namespace local
            {
                class scheduler { public: using service_tag = void; };
                class thread_pool { public: using service_tag = void; };
                class memory_pool { public: using service_tag = void; };
                class watchdog { public: using service_tag = void; };
            }

            namespace remote
            {
                class scheduler { public: using service_tag = void; };
                class thread_pool { public: using service_tag = void; };
                class memory_pool { public: using service_tag = void; };
                class watchdog { public: using service_tag = void; };
            }
        }

        /// @namespace aether::foundation::config
        /// @brief Runtime configuration and parameter management.
        namespace config
        {
            struct parameter {};     ///< @brief A single tunable value with metadata (min/max/default).

            /// @brief Central database for hot-reloadable settings (Contract).
            template<typename T> concept registry = async_service<T>;

            /// @brief Manages mission-specific configuration presets (Contract).
            template<typename T> concept profile_loader = async_service<T>;

            namespace local
            {
                class registry { public: using service_tag = void; };
                class profile_loader { public: using service_tag = void; };
            }

            namespace remote
            {
                class registry { public: using service_tag = void; };
                class profile_loader { public: using service_tag = void; };
            }
        }

        /// @namespace aether::foundation::math
        /// @brief Mathematical primitives and signal processing kernels.
        namespace math
        {
            struct vec3;          ///< @brief 3D Vector primitive.
            struct quaternion {};    ///< @brief Rotation quaternion.
            class kalman_core {};    ///< @brief Generic matrix operations for Kalman filtering.
            class geospatial {};     ///< @brief WGS84 and Geoid coordinate conversions.
        }

        /// @namespace aether::foundation::compute
        /// @brief Hardware accelerated computing interfaces (NPU/GPU).
        namespace compute
        {
            /// @brief Abstraction for TensorRT/EdgeTPU inference (Contract).
            template<typename T> concept neural_runtime = async_service<T>;

            /// @brief Shared context for CUDA/OpenCL operations (Contract).
            template<typename T> concept gpu_context = async_service<T>;

            namespace local
            {
                class neural_runtime { public: using service_tag = void; };
                class gpu_context { public: using service_tag = void; };
            }

            namespace remote
            {
                class neural_runtime { public: using service_tag = void; };
                class gpu_context { public: using service_tag = void; };
            }
        }

        /// @namespace aether::foundation::time
        /// @brief Temporal logic and high-precision clock synchronization.
        namespace time
        {
            struct timestamp {};     ///< @brief Nanosecond-precision PTP timestamp.

            /// @brief IEEE 802.1AS Master/Slave synchronization logic (Contract).
            template<typename T> concept clock_sync = async_service<T>;

            namespace local
            {
                class clock_sync { public: using service_tag = void; };
            }

            namespace remote
            {
                class clock_sync { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::middleware
    /// @brief The communication fabric, security, and data logging layer.
    namespace middleware
    {
        /// @namespace aether::middleware::tsn
        /// @brief Time-Sensitive Networking (IEEE 1722) transport layer.
        namespace tsn
        {
            /// @brief AF_XDP raw socket wrapper (Contract).
            template<typename T> concept interface = async_service<T>;

            /// @brief Zero-copy Publisher/Subscriber pattern implementation (Contract).
            template<typename T> concept pub_sub = async_service<T>;

            /// @brief Bandwidth shaping and traffic prioritization (Contract).
            template<typename T> concept qos_policy = async_service<T>;

            namespace local
            {
                class interface { public: using service_tag = void; };
                class pub_sub { public: using service_tag = void; };
                class qos_policy { public: using service_tag = void; };
            }

            namespace remote
            {
                class interface { public: using service_tag = void; };
                class pub_sub { public: using service_tag = void; };
                class qos_policy { public: using service_tag = void; };
            }
        }

        /// @namespace aether::middleware::security
        /// @brief Cyber-security, encryption, and authentication.
        namespace security
        {
            /// @brief Hardware Secure Element (HSE) abstraction (Contract).
            template<typename T> concept crypto_engine = async_service<T>;

            /// @brief Command signature verification (Contract).
            template<typename T> concept authenticator = async_service<T>;

            /// @brief Secure storage for AES/RSA keys (Contract).
            template<typename T> concept key_store = async_service<T>;

            namespace local
            {
                class crypto_engine { public: using service_tag = void; };
                class authenticator { public: using service_tag = void; };
                class key_store { public: using service_tag = void; };
            }

            namespace remote
            {
                class crypto_engine { public: using service_tag = void; };
                class authenticator { public: using service_tag = void; };
                class key_store { public: using service_tag = void; };
            }
        }

        /// @namespace aether::middleware::io
        /// @brief Persistent storage and diagnostics logging.
        namespace io
        {
            /// @brief High-frequency binary flight data recorder (Contract).
            template<typename T> concept blackbox = async_service<T>;

            /// @brief Text-based system event logging (Contract).
            template<typename T> concept journal = async_service<T>;

            namespace local
            {
                class blackbox { public: using service_tag = void; };
                class journal { public: using service_tag = void; };
            }

            namespace remote
            {
                class blackbox { public: using service_tag = void; };
                class journal { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::sense
    /// @brief Hardware Abstraction Layer (HAL) for sensors and perception inputs.
    namespace sense
    {
        /// @namespace aether::sense::spatial
        /// @brief Sensors related to kinematics, position, and orientation.
        namespace spatial
        {
            /// @brief Redundant IMU management and voting logic (Contract).
            template<typename T> concept imu_array = async_service<T>;

            /// @brief Multi-constellation GNSS interface (Contract).
            template<typename T> concept gnss_receiver = async_service<T>;

            /// @brief Barometric or Laser altimetry (Contract).
            template<typename T> concept altimeter = async_service<T>;

            namespace local
            {
                class imu_array { public: using service_tag = void; };
                class gnss_receiver { public: using service_tag = void; };
                class altimeter { public: using service_tag = void; };
            }

            namespace remote
            {
                class imu_array { public: using service_tag = void; };
                class gnss_receiver { public: using service_tag = void; };
                class altimeter { public: using service_tag = void; };
            }
        }

        /// @namespace aether::sense::environment
        /// @brief Sensors related to the physical medium (Air/Water).
        namespace environment
        {
            /// @brief Pitot/Static systems for airspeed and AoA (Contract).
            template<typename T> concept air_data = async_service<T>;

            /// @brief DVL, Sonar, and Depth sensors (Contract).
            template<typename T> concept ocean_data = async_service<T>;

            /// @brief Short-range obstacle detection (Ultrasonic/ToF) (Contract).
            template<typename T> concept proximity = async_service<T>;

            namespace local
            {
                class air_data { public: using service_tag = void; };
                class ocean_data { public: using service_tag = void; };
                class proximity { public: using service_tag = void; };
            }

            namespace remote
            {
                class air_data { public: using service_tag = void; };
                class ocean_data { public: using service_tag = void; };
                class proximity { public: using service_tag = void; };
            }
        }

        /// @namespace aether::sense::perception
        /// @brief High-bandwidth vision and ranging sensors.
        namespace perception
        {
            struct point_cloud {
                int id = 0;
                float nearest_obstacle_dist = 10.0f;
            };
            struct lidar_config { int rpm; };

            /// @brief Interface for optical and thermal cameras (Contract).
            template<typename T> concept camera = async_service<T>;

            /// @brief Interface for 2D/3D Lidar scanners (Contract).
            template<typename T> concept lidar = async_service<T> && requires(T t, lidar_config c) {
                { t.configure(c) } -> sender_of<void>;
                { t.get_latest_scan() } -> sender_of<point_cloud>;
            };

            /// @brief Interface for automotive/marine radar units (Contract).
            template<typename T> concept radar = async_service<T>;

            /// @brief Edge-AI processing for feature points (Contract).
            template<typename T> concept feature_extractor = async_service<T>;

            /// @brief Stereo-vision depth map generation (Contract).
            template<typename T> concept depth_estimator = async_service<T>;

            namespace local
            {
                class camera { public: using service_tag = void; };
                class lidar {
                public:
                    using service_tag = void;
                    task<void> configure(lidar_config) { co_return; }
                    task<point_cloud> get_latest_scan() {
                        static int counter = 0;
                        point_cloud scan;
                        scan.id = counter++;
                        scan.nearest_obstacle_dist = std::max(0.0f, 10.0f - counter * 1.5f);
                        co_return scan;
                    }
                };
                class radar { public: using service_tag = void; };
                class feature_extractor { public: using service_tag = void; };
                class depth_estimator { public: using service_tag = void; };
            }

            namespace remote
            {
                class camera { public: using service_tag = void; };
                class lidar {
                public:
                    using service_tag = void;
                    // Remote implementation would serialize args here
                    auto configure(lidar_config) { return task_stub<void>{}; }
                    auto get_latest_scan() { return task_stub<point_cloud>{}; }
                };
                class radar { public: using service_tag = void; };
                class feature_extractor { public: using service_tag = void; };
                class depth_estimator { public: using service_tag = void; };
            }
        }

        /// @namespace aether::sense::diagnostics
        /// @brief Vehicle health monitoring sensors.
        namespace diagnostics
        {
            /// @brief FFT analysis for motor/frame integrity (Contract).
            template<typename T> concept vibration_sensor = async_service<T>;

            /// @brief Thermal and load monitoring for compute modules (Contract).
            template<typename T> concept chip_monitor = async_service<T>;

            namespace local
            {
                class vibration_sensor { public: using service_tag = void; };
                class chip_monitor { public: using service_tag = void; };
            }

            namespace remote
            {
                class vibration_sensor { public: using service_tag = void; };
                class chip_monitor { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::motion
    /// @brief Hardware Abstraction Layer (HAL) for actuators and propulsion.
    namespace motion
    {
        /// @namespace aether::motion::propulsion
        /// @brief Force generation mechanisms.
        namespace propulsion
        {
            struct EngineState { float rpm; float temp_c; float voltage; };

            /// @brief Electronic Speed Controller interface (Contract).
            template<typename T> concept esc_driver = async_service<T>;

            /// @brief ICE Throttle, Ignition, and Starter control (Contract).
            template<typename T> concept engine_control = async_service<T> && requires(T t, float nm) {
                { t.arm() } -> sender_of<void>;
                { t.disarm() } -> sender_of<void>;
                { t.set_torque(nm) } -> sender_of<void>;
                { t.get_telemetry() } -> sender_of<EngineState>;
            };

            /// @brief Gimbal or Vectored Thrust control (Contract).
            template<typename T> concept thrust_vector = async_service<T>;

            /// @brief Helicopter swashplate or variable prop control (Contract).
            template<typename T> concept variable_pitch = async_service<T>;

            namespace local
            {
                class esc_driver { public: using service_tag = void; };
                class engine_control {
                public:
                    using service_tag = void;
                    task<void> arm() { co_return; }
                    task<void> disarm() { co_return; }
                    task<void> set_torque(float) { co_return; }
                    task<EngineState> get_telemetry() { co_return EngineState{}; }
                };
                class thrust_vector { public: using service_tag = void; };
                class variable_pitch { public: using service_tag = void; };
            }

            namespace remote
            {
                class esc_driver { public: using service_tag = void; };
                class engine_control {
                public:
                    using service_tag = void;
                    auto arm() { return task_stub<void>{}; }
                    auto disarm() { return task_stub<void>{}; }
                    auto set_torque(float) { return task_stub<void>{}; }
                    auto get_telemetry() { return task_stub<EngineState>{}; }
                };
                class thrust_vector { public: using service_tag = void; };
                class variable_pitch { public: using service_tag = void; };
            }
        }

        /// @namespace aether::motion::articulation
        /// @brief Shape changing and control surface mechanisms.
        namespace articulation
        {
            enum class LightMode { OFF, SOLID, BLINK_SLOW, BLINK_FAST, STROBE };
            struct Color { std::uint8_t r, g, b; };

            /// @brief Manager for PWM/CAN/UAVCAN servos (Contract).
            template<typename T> concept servo_array = async_service<T> && requires(T t, int ch, float angle) {
                { t.set_angle(ch, angle) } -> sender_of<void>;
                { t.set_limits(ch, angle, angle) } -> sender_of<void>;
            };

            /// @brief Retractable gear logic (Contract).
            template<typename T> concept landing_gear = async_service<T>;

            /// @brief Navigation lights and strobes (Contract).
            template<typename T> concept lighting = async_service<T> && requires(T t, LightMode m, Color c) {
                { t.set_mode(m) } -> sender_of<void>;
                { t.set_color(c) } -> sender_of<void>;
            };

            namespace local
            {
                class servo_array {
                public:
                    using service_tag = void;
                    task<void> set_angle(int, float) { co_return; }
                    task<void> set_limits(int, float, float) { co_return; }
                };
                class landing_gear { public: using service_tag = void; };
                class lighting {
                public:
                    using service_tag = void;
                    task<void> set_mode(LightMode) { co_return; }
                    task<void> set_color(Color) { co_return; }
                };
            }

            namespace remote
            {
                class servo_array {
                public:
                    using service_tag = void;
                    auto set_angle(int, float) { return task_stub<void>{}; }
                    auto set_limits(int, float, float) { return task_stub<void>{}; }
                };
                class landing_gear { public: using service_tag = void; };
                class lighting {
                public:
                    using service_tag = void;
                    auto set_mode(LightMode) { return task_stub<void>{}; }
                    auto set_color(Color) { return task_stub<void>{}; }
                };
            }
        }

        /// @namespace aether::motion::power
        /// @brief Energy management and distribution.
        namespace power
        {
            /// @brief Battery Management System interface (Contract).
            template<typename T> concept bms_manager = async_service<T>;

            /// @brief Smart switching for power rails (Contract).
            template<typename T> concept pdu_channel = async_service<T>;

            namespace local
            {
                class bms_manager { public: using service_tag = void; };
                class pdu_channel { public: using service_tag = void; };
            }

            namespace remote
            {
                class bms_manager { public: using service_tag = void; };
                class pdu_channel { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::gnc
    /// @brief Guidance, Navigation, and Control (The Autonomy Stack).
    namespace gnc
    {
        /// @namespace aether::gnc::navigation
        /// @brief State estimation and localization.
        namespace navigation
        {
            /// @brief Extended Kalman Filter for state estimation (Contract).
            template<typename T> concept ekf_fusion = async_service<T>;

            /// @brief Simultaneous Localization and Mapping logic (Contract).
            template<typename T> concept slam_engine = async_service<T>;

            /// @brief Management of ECEF reference and Home point (Contract).
            template<typename T> concept origin_manager = async_service<T>;

            namespace local
            {
                class ekf_fusion { public: using service_tag = void; };
                class slam_engine { public: using service_tag = void; };
                class origin_manager { public: using service_tag = void; };
            }

            namespace remote
            {
                class ekf_fusion { public: using service_tag = void; };
                class slam_engine { public: using service_tag = void; };
                class origin_manager { public: using service_tag = void; };
            }
        }

        /// @namespace aether::gnc::guidance
        /// @brief Path planning and trajectory generation.
        namespace guidance
        {
            /// @brief Kinematic trajectory generation (Jerk-limited) (Contract).
            template<typename T> concept trajectory_gen = async_service<T>;

            /// @brief Map-based pathfinding (A*, Dijkstra) (Contract).
            template<typename T> concept global_planner = async_service<T>;

            /// @brief Reactive obstacle avoidance (VFH+, DWA) (Contract).
            template<typename T> concept local_planner = async_service<T>;

            /// @brief Formation keeping and inter-agent spacing (Contract).
            template<typename T> concept swarm_logic = async_service<T>;

            namespace local
            {
                class trajectory_gen { public: using service_tag = void; };
                class global_planner {
                    scheduler* _sched = nullptr;
                public:
                    using service_tag = void;
                    void set_scheduler(scheduler& s) { _sched = &s; }

                    task<float> make_plan() {
                         if (_sched) co_await sleep_stub(*_sched);
                         co_return 10.0f;
                    }
                };

                class local_planner {
                    scheduler* _sched = nullptr;
                public:
                    using service_tag = void;
                    void set_scheduler(scheduler& s) { _sched = &s; }

                    task<float> compute_safe_cmd(float target_speed, sense::perception::point_cloud scan) {
                         if (_sched) co_await sleep_stub(*_sched);

                         if (scan.nearest_obstacle_dist < 2.0f) co_return 0.0f;
                         co_return target_speed;
                    }
                };

                class swarm_logic { public: using service_tag = void; };
            }

            namespace remote
            {
                class trajectory_gen { public: using service_tag = void; };
                class global_planner { public: using service_tag = void; };
                class local_planner { public: using service_tag = void; };
                class swarm_logic { public: using service_tag = void; };
            }
        }

        /// @namespace aether::gnc::control
        /// @brief Loop stability and output mixing.
        namespace control
        {
            /// @brief Inner-loop rate and angle controllers (Contract).
            template<typename T> concept attitude_ctrl = async_service<T>;

            /// @brief Outer-loop velocity and position controllers (Contract).
            template<typename T> concept position_ctrl = async_service<T>;

            /// @brief Control Allocation Matrix (The "Mixer") (Contract).
            template<typename T> concept allocator = async_service<T>;

            namespace local
            {
                class attitude_ctrl { public: using service_tag = void; };
                class position_ctrl { public: using service_tag = void; };
                class allocator { public: using service_tag = void; };
            }

            namespace remote
            {
                class attitude_ctrl { public: using service_tag = void; };
                class position_ctrl { public: using service_tag = void; };
                class allocator { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::assurance
    /// @brief Mission Assurance, Safety, and Fault Management.
    namespace assurance
    {
        /// @namespace aether::assurance::fdir
        /// @brief Fault Detection, Isolation, and Recovery.
        namespace fdir
        {
            /// @brief Statistical variance and timeout monitoring (Contract).
            template<typename T> concept fault_detector = async_service<T>;

            /// @brief Redundancy switching logic (e.g., GPS -> Optical Flow) (Contract).
            template<typename T> concept failover_logic = async_service<T>;

            /// @brief Terminal safety procedures (Chute, Flight Termination) (Contract).
            template<typename T> concept emergency_proc = async_service<T>;

            namespace local
            {
                class fault_detector { public: using service_tag = void; };
                class failover_logic { public: using service_tag = void; };
                class emergency_proc { public: using service_tag = void; };
            }

            namespace remote
            {
                class fault_detector { public: using service_tag = void; };
                class failover_logic { public: using service_tag = void; };
                class emergency_proc { public: using service_tag = void; };
            }
        }

        /// @namespace aether::assurance::state_machine
        /// @brief High-level vehicle state management.
        namespace state_machine
        {
            /// @brief Transitions between Manual, Auto, Stabilize, etc. (Contract).
            template<typename T> concept mode_manager = async_service<T>;

            /// @brief Arming constraints and safety interlocks (Contract).
            template<typename T> concept preflight_check = async_service<T>;

            namespace local
            {
                class mode_manager { public: using service_tag = void; };
                class preflight_check { public: using service_tag = void; };
            }

            namespace remote
            {
                class mode_manager { public: using service_tag = void; };
                class preflight_check { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::payload
    /// @brief Mission-specific capabilities distinct from vehicle flight.
    namespace payload
    {
        /// @namespace aether::payload::logistics
        /// @brief Material handling and delivery systems.
        namespace logistics
        {
            /// @brief Claws, magnets, and retention systems (Contract).
            template<typename T> concept gripper_mech = async_service<T> && requires(T t) {
                { t.grab() } -> sender_of<bool>;
                { t.release() } -> sender_of<void>;
                { t.get_force() } -> sender_of<float>;
            };

            /// @brief Cable delivery logic (Contract).
            template<typename T> concept winch_control = async_service<T>;

            /// @brief Fluid flow control for agriculture/firefighting (Contract).
            template<typename T> concept pump_system = async_service<T>;

            namespace local
            {
                class gripper_mech {
                public:
                     using service_tag = void;
                     task<bool> grab() { co_return bool{}; }
                     task<void> release() { co_return; }
                     task<float> get_force() { co_return float{}; }
                };
                class winch_control { public: using service_tag = void; };
                class pump_system { public: using service_tag = void; };
            }

            namespace remote
            {
                class gripper_mech {
                public:
                     using service_tag = void;
                     auto grab() { return task_stub<bool>{}; }
                     auto release() { return task_stub<void>{}; }
                     auto get_force() { return task_stub<float>{}; }
                };
                class winch_control { public: using service_tag = void; };
                class pump_system { public: using service_tag = void; };
            }
        }

        /// @namespace aether::payload::tactical
        /// @brief Kinetic and Electronic effectors.
        namespace tactical
        {
            /// @brief Critical safety interlock for dangerous payloads (Contract).
            template<typename T> concept master_arm = async_service<T>;

            /// @brief Generic interface for deployable munitions (Contract).
            template<typename T> concept weapon_system = async_service<T>;

            /// @brief Gimballed sensor suite for designation (Contract).
            template<typename T> concept targeting_pod = async_service<T>;

            /// @brief Electronic Warfare (Jamming/Sensing) (Contract).
            template<typename T> concept ew_suite = async_service<T>;

            namespace local
            {
                class master_arm { public: using service_tag = void; };
                class weapon_system { public: using service_tag = void; };
                class targeting_pod { public: using service_tag = void; };
                class ew_suite { public: using service_tag = void; };
            }

            namespace remote
            {
                class master_arm { public: using service_tag = void; };
                class weapon_system { public: using service_tag = void; };
                class targeting_pod { public: using service_tag = void; };
                class ew_suite { public: using service_tag = void; };
            }
        }
    }

    /// @namespace aether::telematics
    /// @brief Communications and remote control.
    namespace telematics
    {
        namespace link
        {
            /// @brief Mavlink Bridge for GCS communication (Contract).
            template<typename T> concept mavlink_bridge = async_service<T>;

            /// @brief Satellite communication modem (Contract).
            template<typename T> concept satcom_modem = async_service<T>;

            /// @brief RCEM Receiver for pilot input (Contract).
            template<typename T> concept rcem_receiver = async_service<T>;

            namespace local
            {
                class mavlink_bridge { public: using service_tag = void; };
                class satcom_modem { public: using service_tag = void; };
                class rcem_receiver { public: using service_tag = void; };
            }

            namespace remote
            {
                class mavlink_bridge { public: using service_tag = void; };
                class satcom_modem { public: using service_tag = void; };
                class rcem_receiver { public: using service_tag = void; };
            }
        }
    }
}

