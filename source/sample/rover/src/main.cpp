#include <iostream>
#include <type_traits>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::rover
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace payload = kmx::aether::v0_1::payload;
    namespace gnc = kmx::aether::v0_1::gnc;
    namespace assurance = kmx::aether::v0_1::assurance;
    namespace telematics = kmx::aether::v0_1::telematics;
    namespace sys = kmx::aether::v0_1::foundation::sys;

    template<
        sense::perception::Lidar lidar_t,
        motion::propulsion::EngineControl engine_t,
        motion::articulation::ServoArray servo_t,
        payload::logistics::GripperMech gripper_t,
        motion::articulation::Lighting lighting_t
    >
    struct hardware_t
    {
        lidar_t lidar;
        engine_t motors;
        servo_t steering; // Steerable wheels
        gripper_t claw;

        lighting_t car_signal_lights;
        lighting_t approach_distance_lights;
        lighting_t stop_lights;
        lighting_t rear_signal_lights;
    };

    template<
        gnc::navigation::EkfFusion ekf_t,
        gnc::navigation::SlamEngine slam_t,
        gnc::guidance::GlobalPlanner global_planner_t,
        gnc::guidance::LocalPlanner local_planner_t,
        gnc::control::PositionCtrl position_ctrl_t
    >
    struct autonomy_t
    {
        ekf_t estimator;       // Determines "Where am I?"
        slam_t mapper;         // Builds map from Lidar
        global_planner_t path_finder;   // A* / Dijkstra (Long range)
        local_planner_t obstacle_avoid; // Reacts to immediate hazards
        position_ctrl_t controller;      // Drives motors to follow path
    };

    template<
        assurance::fdir::FaultDetector fault_detector_t,
        assurance::state_machine::ModeManager mode_manager_t,
        sys::Watchdog watchdog_t
    >
    struct safety_t
    {
        fault_detector_t health_monitor;
        mode_manager_t mode_switch;
        watchdog_t watchdog_timer;
    };

    template<
        telematics::link::MavlinkBridge mavlink_t
    >
    struct comms_t
    {
        mavlink_t gcs_link;
    };

    template<
        typename hardware_type,
        typename autonomy_type,
        typename safety_type,
        typename comms_type
    >
    struct vehicle_t
    {
        hardware_type hw;
        autonomy_type ai;
        safety_type protection;
        comms_type radio;
    };

    template <bool use_remote>
    struct configurator
    {
        template<typename local_t, typename remote_t>
        using impl_t = std::conditional_t<use_remote, remote_t, local_t>;

        using hardware_config = hardware_t<
            impl_t<sense::perception::local::lidar, sense::perception::remote::lidar>,
            impl_t<motion::propulsion::local::engine_control, motion::propulsion::remote::engine_control>,
            impl_t<motion::articulation::local::servo_array, motion::articulation::remote::servo_array>,
            impl_t<payload::logistics::local::gripper_mech, payload::logistics::remote::gripper_mech>,
            impl_t<motion::articulation::local::lighting, motion::articulation::remote::lighting>
        >;

        using autonomy_config = autonomy_t<
            impl_t<gnc::navigation::local::ekf_fusion, gnc::navigation::remote::ekf_fusion>,
            impl_t<gnc::navigation::local::slam_engine, gnc::navigation::remote::slam_engine>,
            impl_t<gnc::guidance::local::global_planner, gnc::guidance::remote::global_planner>,
            impl_t<gnc::guidance::local::local_planner, gnc::guidance::remote::local_planner>,
            impl_t<gnc::control::local::position_ctrl, gnc::control::remote::position_ctrl>
        >;

        using safety_config = safety_t<
            impl_t<assurance::fdir::local::fault_detector, assurance::fdir::remote::fault_detector>,
            impl_t<assurance::state_machine::local::mode_manager, assurance::state_machine::remote::mode_manager>,
            impl_t<sys::local::watchdog, sys::remote::watchdog>
        >;

        using comms_config = comms_t<
            impl_t<telematics::link::local::mavlink_bridge, telematics::link::remote::mavlink_bridge>
        >;

        using vehicle = vehicle_t<hardware_config, autonomy_config, safety_config, comms_config>;
    };
}

int main() noexcept
{
    using namespace kmx::aether::v0_1::sample::rover;

    constexpr bool use_remote = false; // Set to true to switch to remote implementations

    // Instantiate the complete vehicle (false = local, true = remote)
    using rover_vehicle = configurator<use_remote>::vehicle;

    try
    {
        rover_vehicle rover;
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cerr << "unknown exception" << std::endl;
        return -2;
    }

    return 0;
}
