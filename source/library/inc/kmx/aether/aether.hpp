/// @file aether.hpp
/// @brief Aether: Unified Modular Architecture for Multi-Domain Unmanned Vehicles.
/// @details This module defines the high-level namespace topology for the Aether system,
///          targeting Air, Land, and Sea domains using C++26 and IEEE 1722 TSN.
/// @version 0.1
/// @copyright KMX Systems

/// @namespace aether
/// @brief The root namespace for the unified architecture.
namespace kmx::aether::v0_1
{
    /// @namespace aether::foundation
    /// @brief The bedrock layer providing low-level system abstractions, mathematics, and runtime utilities.
    namespace foundation
    {
        /// @namespace aether::foundation::sys
        /// @brief Operating System primitives and execution models.
        namespace sys
        {
            class scheduler {};      ///< @brief Coroutine executor and task dispatcher.
            class thread_pool {};    ///< @brief Worker thread management for parallel execution.
            class memory_pool {};    ///< @brief Real-time polymorphic memory resources (PMR).
            class watchdog {};       ///< @brief Hardware independent watchdog timer interface.
        }

        /// @namespace aether::foundation::config
        /// @brief Runtime configuration and parameter management.
        namespace config
        {
            struct parameter {};     ///< @brief A single tunable value with metadata (min/max/default).
            class registry {};       ///< @brief Central database for hot-reloadable settings.
            class profile_loader {}; ///< @brief Manages mission-specific configuration presets.
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
            class neural_runtime {}; ///< @brief Abstraction for TensorRT/EdgeTPU inference.
            class gpu_context {};    ///< @brief Shared context for CUDA/OpenCL operations.
        }

        /// @namespace aether::foundation::time
        /// @brief Temporal logic and high-precision clock synchronization.
        namespace time
        {
            struct timestamp {};     ///< @brief Nanosecond-precision PTP timestamp.
            class clock_sync {};     ///< @brief IEEE 802.1AS Master/Slave synchronization logic.
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
            class interface {};      ///< @brief AF_XDP raw socket wrapper.
            class pub_sub {};        ///< @brief Zero-copy Publisher/Subscriber pattern implementation.
            class qos_policy {};     ///< @brief Bandwidth shaping and traffic prioritization.
        }

        /// @namespace aether::middleware::security
        /// @brief Cyber-security, encryption, and authentication.
        namespace security
        {
            class crypto_engine {};  ///< @brief Hardware Secure Element (HSE) abstraction.
            class authenticator {};  ///< @brief Command signature verification.
            class key_store {};      ///< @brief Secure storage for AES/RSA keys.
        }

        /// @namespace aether::middleware::io
        /// @brief Persistent storage and diagnostics logging.
        namespace io
        {
            class blackbox {};       ///< @brief High-frequency binary flight data recorder.
            class journal {};        ///< @brief Text-based system event logging.
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
            class imu_array {};      ///< @brief Redundant IMU management and voting logic.
            class gnss_receiver {};  ///< @brief Multi-constellation GNSS interface.
            class altimeter {};      ///< @brief Barometric or Laser altimetry.
        }

        /// @namespace aether::sense::environment
        /// @brief Sensors related to the physical medium (Air/Water).
        namespace environment
        {
            class air_data {};       ///< @brief Pitot/Static systems for airspeed and AoA.
            class ocean_data {};     ///< @brief DVL, Sonar, and Depth sensors.
            class proximity {};      ///< @brief Short-range obstacle detection (Ultrasonic/ToF).
        }

        /// @namespace aether::sense::perception
        /// @brief High-bandwidth vision and ranging sensors.
        namespace perception
        {
            class camera {};            ///< @brief Interface for optical and thermal cameras.
            class lidar {};             ///< @brief Interface for 2D/3D Lidar scanners.
            class radar {};             ///< @brief Interface for automotive/marine radar units.
            class feature_extractor {}; ///< @brief Edge-AI processing for feature points.
            class depth_estimator {};   ///< @brief Stereo-vision depth map generation.
        }

        /// @namespace aether::sense::diagnostics
        /// @brief Vehicle health monitoring sensors.
        namespace diagnostics
        {
            class vibration_sensor {}; ///< @brief FFT analysis for motor/frame integrity.
            class chip_monitor {};     ///< @brief Thermal and load monitoring for compute modules.
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
            class esc_driver {};     ///< @brief Electronic Speed Controller interface.
            class engine_control {}; ///< @brief ICE Throttle, Ignition, and Starter control.
            class thrust_vector {};  ///< @brief Gimbal or Vectored Thrust control.
            class variable_pitch {}; ///< @brief Helicopter swashplate or variable prop control.
        }

        /// @namespace aether::motion::articulation
        /// @brief Shape changing and control surface mechanisms.
        namespace articulation
        {
            class servo_array {};    ///< @brief Manager for PWM/CAN/UAVCAN servos.
            class landing_gear {};   ///< @brief Retractable gear logic.
            class lighting {};       ///< @brief Navigation lights and strobes.
        }

        /// @namespace aether::motion::power
        /// @brief Energy management and distribution.
        namespace power
        {
            class bms_manager {};    ///< @brief Battery Management System interface.
            class pdu_channel {};    ///< @brief Smart switching for power rails.
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
            class ekf_fusion {};     ///< @brief Extended Kalman Filter for state estimation.
            class slam_engine {};    ///< @brief Simultaneous Localization and Mapping logic.
            class origin_manager {}; ///< @brief Management of ECEF reference and Home point.
        }

        /// @namespace aether::gnc::guidance
        /// @brief Path planning and trajectory generation.
        namespace guidance
        {
            class trajectory_gen {}; ///< @brief Kinematic trajectory generation (Jerk-limited).
            class global_planner {}; ///< @brief Map-based pathfinding (A*, Dijkstra).
            class local_planner {};  ///< @brief Reactive obstacle avoidance (VFH+, DWA).
            class swarm_logic {};    ///< @brief Formation keeping and inter-agent spacing.
        }

        /// @namespace aether::gnc::control
        /// @brief Loop stability and output mixing.
        namespace control
        {
            class attitude_ctrl {};  ///< @brief Inner-loop rate and angle controllers.
            class position_ctrl {};  ///< @brief Outer-loop velocity and position controllers.
            class allocator {};      ///< @brief Control Allocation Matrix (The "Mixer").
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
            class fault_detector {}; ///< @brief Statistical variance and timeout monitoring.
            class failover_logic {}; ///< @brief Redundancy switching logic (e.g., GPS -> Optical Flow).
            class emergency_proc {}; ///< @brief Terminal safety procedures (Chute, Flight Termination).
        }

        /// @namespace aether::assurance::state_machine
        /// @brief High-level vehicle state management.
        namespace state_machine
        {
            class mode_manager {};    ///< @brief Transitions between Manual, Auto, Stabilize, etc.
            class preflight_check {}; ///< @brief Arming constraints and safety interlocks.
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
            class gripper_mech {};   ///< @brief Claws, magnets, and retention systems.
            class winch_control {};  ///< @brief Cable delivery logic.
            class pump_system {};    ///< @brief Fluid flow control for agriculture/firefighting.
        }

        /// @namespace aether::payload::tactical
        /// @brief Kinetic and Electronic effectors.
        namespace tactical
        {
            class master_arm {};     ///< @brief Critical safety interlock for dangerous payloads.
            class weapon_system {};  ///< @brief Generic interface for deployable munitions.
            class targeting_pod {};  ///< @brief Gimballed sensor suite for designation.
            class ew_suite {};       ///< @brief Electronic Warfare (Jamming/Sensing).
        }
    }

    /// @namespace aether::telematics
    /// @brief External connectivity and remote command links.
    namespace telematics
    {
        /// @namespace aether::telematics::link
        /// @brief Data link protocols and bridging.
        namespace link
        {
            class mavlink_bridge {}; ///< @brief Bridge to GCS using MAVLink protocol.
            class dds_bridge {};     ///< @brief Bridge to DDS/ROS2 networks.
            class rcem_receiver {};  ///< @brief Pilot Input receiver (RC/EPM/PPM).
            class satcom_modem {};   ///< @brief High-latency satellite communication handler.
        }

        /// @namespace aether::telematics::media
        /// @brief Audio/Video streaming and encoding.
        namespace media
        {
            class video_encoder {};  ///< @brief Hardware-accelerated video compression.
            class stream_server {};  ///< @brief RTSP/WebRTC streaming server.
        }
    }
}
