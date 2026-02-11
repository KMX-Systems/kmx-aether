#include <iostream>
#include <type_traits>
#include <mutex>
#include <csignal>
#include <atomic>
#include <kmx/aether/aether.hpp>
#include <kmx/aether/scheduler.hpp>

namespace kmx::aether::v0_1::sample::rover
{
    std::atomic<bool> g_keep_running{true};

    void handle_signal(int signal)
    {
        if (signal == SIGINT)
        {
            g_keep_running = false;
        }
    }

    std::mutex g_io_mutex;

    void safe_print(const auto&... args)
    {
        std::lock_guard lock(g_io_mutex);
        ((std::cout << args), ...) << std::endl;
    }

    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace payload = kmx::aether::v0_1::payload;
    namespace gnc = kmx::aether::v0_1::gnc;
    namespace assurance = kmx::aether::v0_1::assurance;
    namespace telematics = kmx::aether::v0_1::telematics;
    namespace sys = kmx::aether::v0_1::foundation::sys;

    template<
        sense::perception::lidar lidar_t,
        motion::propulsion::engine_control engine_t,
        motion::articulation::servo_array servo_t,
        payload::logistics::gripper_mech gripper_t,
        motion::articulation::lighting lighting_t
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
        gnc::navigation::ekf_fusion ekf_t,
        gnc::navigation::slam_engine slam_t,
        gnc::guidance::global_planner global_planner_t,
        gnc::guidance::local_planner local_planner_t,
        gnc::control::position_ctrl position_ctrl_t
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
        assurance::fdir::fault_detector fault_detector_t,
        assurance::state_machine::mode_manager mode_manager_t,
        sys::watchdog watchdog_t
    >
    struct safety_t
    {
        fault_detector_t health_monitor;
        mode_manager_t mode_switch;
        watchdog_t watchdog_timer;
    };

    template<
        telematics::link::mavlink_bridge mavlink_t
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

    template<typename Vehicle>
    kmx::aether::v0_1::task<void> mission(Vehicle& rover, kmx::aether::v0_1::scheduler& sched)
    {
        using namespace kmx::aether::v0_1;

        co_await rover.hw.motors.arm();
        safe_print("System Armed");

        for (int i = 0; g_keep_running; ++i)
        {
            safe_print("[Mission] Iteration ", i, ": Sensing...");
            auto scan = co_await rover.hw.lidar.get_latest_scan();
            safe_print("  > LiDAR: Scan ID #", scan.id, " | Dist: ", scan.nearest_obstacle_dist, "m");

            // 1. Global
            safe_print("  > Global Planner: Generating route...");
            const auto desired_speed = co_await rover.ai.path_finder.make_plan();

            // 2. Local
            safe_print("  > Local Planner: Checking obstacles...");
            const auto safe_torque = co_await rover.ai.obstacle_avoid.compute_safe_cmd(desired_speed, scan);
            safe_print("  > Local Planner: Safe Command: ", safe_torque, " Nm");

            // 3. Actuate
            co_await rover.hw.motors.set_torque(safe_torque);

            safe_print("[Mission] Resting...");
            co_await sleep_stub(sched);
        }

        safe_print("Mission Complete");
        co_await rover.hw.motors.disarm();
        sched.stop();
    }
}

int main() noexcept
{
    using namespace kmx::aether::v0_1;
    using namespace kmx::aether::v0_1::sample::rover;

    std::signal(SIGINT, handle_signal);

    scheduler sched;

    constexpr bool use_remote = false;
    using rover_vehicle = configurator<use_remote>::vehicle;
    rover_vehicle rover;

    rover.ai.path_finder.set_scheduler(sched);
    rover.ai.obstacle_avoid.set_scheduler(sched);

    auto m = mission(rover, sched);
    sched.schedule(m.handle);
    sched.run();

    return 0;
}
