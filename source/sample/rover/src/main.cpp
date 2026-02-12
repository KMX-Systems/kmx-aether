#include <iostream>
#include <type_traits>
#include <mutex>
#include <csignal>
#include <atomic>
#include <random>
#include <thread>
#include <chrono>
#include <optional>
#include <cmath>
#include <algorithm>
#include <vector>
#include <span>
#include <kmx/aether/aether.hpp>
#include <kmx/aether/scheduler.hpp>

namespace kmx::aether::v0_1::sample::rover
{
    std::atomic<bool> g_keep_running{true};

    void handle_signal(int signal) noexcept
    {
        if (signal == SIGINT)
        {
            g_keep_running = false;
        }
    }

    // Random generator helper
    float random_float(float min, float max)
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(min, max);
        return dis(gen);
    }

    std::mutex g_io_mutex;

    void safe_print(const auto&... args)
    {
        std::lock_guard lock(g_io_mutex);
        if constexpr (sizeof...(args) > 0) {
            ((std::cout << args), ...) << std::endl;
        }
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
        sense::spatial::gnss_receiver gps_t,
        sense::spatial::magnetometer mag_t,
        motion::propulsion::engine_control engine_t,
        motion::articulation::servo_array servo_t,
        payload::logistics::gripper_mech gripper_t,
        motion::articulation::lighting lighting_t
    >
    struct hardware_t
    {
        lidar_t lidar;
        gps_t gps;
        mag_t compass;
        engine_t motors;
        servo_t steering; // Steerable wheels
        gripper_t claw;

        lighting_t front_lights;
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

    struct drive_cmd
    {
        float torque;
        float steering_angle;
    };

    struct waypoint
    {
        float x_m;
        float y_m;
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

        task<drive_cmd> execute_navigation_cycle(
            std::size_t mission_iteration,
            const waypoint& current_position,
            float distance_to_target_m);

        void update_position_towards_target(
            waypoint& current_position,
            const waypoint& target,
            float distance_to_target_m,
            float commanded_speed_mps,
            int cycle_time_ms);

        task<bool> run_waypoint_until_reached(
            const waypoint& target,
            waypoint& current_position,
            std::size_t& mission_iteration,
            scheduler& sched);

        task<void> mission(
            std::span<const waypoint> route,
            scheduler& sched);
    };

    template<typename hardware_type, typename autonomy_type, typename safety_type, typename comms_type>
    task<drive_cmd> vehicle_t<hardware_type, autonomy_type, safety_type, comms_type>::execute_navigation_cycle(
        std::size_t mission_iteration,
        const waypoint& current_position,
        float distance_to_target_m)
    {
        static constexpr int steering_channel = 0;

        safe_print("[Mission] Iteration ", mission_iteration,
                   " | Position (", current_position.x_m, ", ", current_position.y_m,
                   ") m | Distance to waypoint: ", distance_to_target_m, " m");

        safe_print("  > Sensing...");
        auto scan = co_await this->hw.lidar.get_latest_scan();
        safe_print("  > LiDAR: Scan ID #", scan.id, " | Dist: ", scan.nearest_obstacle_dist, "m");

        const auto gps_heading = co_await this->hw.gps.get_heading_deg();
        if (gps_heading.has_value())
        {
            safe_print("  > Nav: GPS heading ", *gps_heading, " deg");
        }
        else
        {
            const auto compass_heading = co_await this->hw.compass.get_heading_deg();
            safe_print("  > Nav: GPS unavailable, using compass heading ", compass_heading, " deg");
        }

        safe_print("  > Global Planner: Generating route...");
        const auto desired_speed = co_await this->ai.path_finder.make_plan();

        safe_print("  > Local Planner: Checking obstacles...");
        const auto cmd = co_await this->ai.obstacle_avoid.compute_safe_cmd(desired_speed, scan);
        safe_print("  > Local Planner: Safe Command: ", cmd.torque, " Nm, Steering: ", cmd.steering_angle, " deg");

        co_await this->hw.motors.set_torque(cmd.torque);
        co_await this->hw.steering.set_angle(steering_channel, cmd.steering_angle);

        co_return cmd;
    }

    template<typename hardware_type, typename autonomy_type, typename safety_type, typename comms_type>
    void vehicle_t<hardware_type, autonomy_type, safety_type, comms_type>::update_position_towards_target(
        waypoint& current_position,
        const waypoint& target,
        float distance_to_target_m,
        float commanded_speed_mps,
        int cycle_time_ms)
    {
        const float max_step_m = commanded_speed_mps * (static_cast<float>(cycle_time_ms) / 1000.0f);
        if (max_step_m <= 0.0f)
        {
            return;
        }

        const float dx = target.x_m - current_position.x_m;
        const float dy = target.y_m - current_position.y_m;
        const float step_m = std::min(distance_to_target_m, max_step_m);
        const float inv_dist = 1.0f / distance_to_target_m;
        current_position.x_m += dx * inv_dist * step_m;
        current_position.y_m += dy * inv_dist * step_m;
    }

    template<typename hardware_type, typename autonomy_type, typename safety_type, typename comms_type>
    task<bool> vehicle_t<hardware_type, autonomy_type, safety_type, comms_type>::run_waypoint_until_reached(
        const waypoint& target,
        waypoint& current_position,
        std::size_t& mission_iteration,
        scheduler& sched)
    {
        static constexpr int cycle_time_ms = 200;
        static constexpr float waypoint_reached_distance_m = 0.5f;
        static constexpr int max_cycles_per_waypoint = 200;

        int waypoint_cycles = 0;
        while (g_keep_running)
        {
            const float dx = target.x_m - current_position.x_m;
            const float dy = target.y_m - current_position.y_m;
            const float distance_to_target_m = std::sqrt(dx * dx + dy * dy);

            if (distance_to_target_m <= waypoint_reached_distance_m)
            {
                safe_print("[Mission] Waypoint reached at (", current_position.x_m, ", ", current_position.y_m, ") m");
                co_return true;
            }

            if (waypoint_cycles >= max_cycles_per_waypoint)
            {
                safe_print("[Mission] Waypoint timeout, stopping mission for safety");
                g_keep_running = false;
                co_return false;
            }

            ++waypoint_cycles;
            ++mission_iteration;

            const auto cmd = co_await this->execute_navigation_cycle(mission_iteration, current_position, distance_to_target_m);

            this->update_position_towards_target(
                current_position,
                target,
                distance_to_target_m,
                std::max(0.0f, cmd.torque),
                cycle_time_ms);

            safe_print("  > Updated Position (", current_position.x_m, ", ", current_position.y_m, ") m");
            safe_print("[Mission] Resting...");

            std::this_thread::sleep_for(std::chrono::milliseconds(cycle_time_ms));
            co_await sleep_stub(sched);
        }

        co_return false;
    }

    template<typename hardware_type, typename autonomy_type, typename safety_type, typename comms_type>
    task<void> vehicle_t<hardware_type, autonomy_type, safety_type, comms_type>::mission(
        std::span<const waypoint> route,
        scheduler& sched)
    {
        co_await this->hw.motors.arm();
        safe_print("System Armed");

        if (route.empty())
        {
            safe_print("[Mission] No waypoints provided");
            safe_print("Mission Complete");
            co_await this->hw.motors.disarm();
            sched.stop();
            co_return;
        }

        waypoint current_position{0.0f, 0.0f};
        std::size_t mission_iteration = 0;

        safe_print("[Mission] Current position (", current_position.x_m, ", ", current_position.y_m, ") m");

        for (std::size_t waypoint_index = 0; g_keep_running && waypoint_index < route.size(); ++waypoint_index)
        {
            const auto& target = route[waypoint_index];
            safe_print("[Mission] Waypoint ", waypoint_index + 1, "/", route.size(),
                       " -> Target (", target.x_m, ", ", target.y_m, ") m");

            const bool reached = co_await this->run_waypoint_until_reached(target, current_position, mission_iteration, sched);
            if (!reached)
            {
                break;
            }
        }

        safe_print("Mission Complete");
        co_await this->hw.motors.disarm();
        sched.stop();
    }

    class rover_global_planner
    {
        static constexpr int planning_delay_ms = 200;
        static constexpr float min_speed_mps = 8.0f;
        static constexpr float max_speed_mps = 12.0f;

        scheduler* _sched = nullptr;
    public:
        using service_tag = void;
        void set_scheduler(scheduler& s) noexcept { _sched = &s; }

        task<float> make_plan()
        {
             if (_sched) co_await sleep_stub(*_sched);

             // Simulate heavy A* calculation
             std::this_thread::sleep_for(std::chrono::milliseconds(planning_delay_ms));

             co_return random_float(min_speed_mps, max_speed_mps);
        }
    };

    class rover_local_planner
    {
        static constexpr float min_safe_distance_m = 2.0f;
        static constexpr float avoidance_turn_angle_deg = 15.0f;
        static constexpr int sensor_fusion_delay_ms = 50;
        static constexpr float steering_noise_deg = 2.0f;
        static constexpr float torque_noise_nm = 0.5f;

           scheduler* _sched = nullptr;
    public:
        using service_tag = void;
           void set_scheduler(scheduler& s) noexcept { _sched = &s; }

           task<drive_cmd> compute_safe_cmd(float target_speed, const sense::perception::point_cloud scan)
        {
               if (_sched) co_await sleep_stub(*_sched);

             // Simulate sensor fusion delay
             std::this_thread::sleep_for(std::chrono::milliseconds(sensor_fusion_delay_ms));

             if (scan.nearest_obstacle_dist < min_safe_distance_m)
             {
                 const float noisy_turn = avoidance_turn_angle_deg + random_float(-steering_noise_deg, steering_noise_deg);
                 co_return drive_cmd{0.0f, noisy_turn}; // Stop and turn
             }

             const float noisy_torque = target_speed + random_float(-torque_noise_nm, torque_noise_nm);
             // Add small steering corrections even when going straight
             const float straight_steering = random_float(-steering_noise_deg, steering_noise_deg);
             co_return drive_cmd{noisy_torque, straight_steering}; // Go straight
        }
    };

    class rover_lidar
    {
        static constexpr float min_dist_m = 0.5f;
        static constexpr float max_dist_m = 15.0f;
    public:
        using service_tag = void;

        task<void> configure(sense::perception::lidar_config)
        {
            co_return;
        }

        task<sense::perception::point_cloud> get_latest_scan()
        {
            static int counter = 0;
            sense::perception::point_cloud scan;
            scan.id = counter++;
            // Random distance between 0.5m and 15.0m
            scan.nearest_obstacle_dist = random_float(min_dist_m, max_dist_m);
            co_return scan;
        }
    };

    class rover_gnss_receiver
    {
        static constexpr float min_heading_deg = 0.0f;
        static constexpr float max_heading_deg = 359.9f;
        static constexpr float gps_fix_probability = 0.8f;
    public:
        using service_tag = void;

        task<std::optional<float>> get_heading_deg()
        {
            if (random_float(0.0f, 1.0f) <= gps_fix_probability)
            {
                co_return random_float(min_heading_deg, max_heading_deg);
            }

            co_return std::nullopt;
        }
    };

    class rover_magnetometer
    {
        static constexpr float min_heading_deg = 0.0f;
        static constexpr float max_heading_deg = 359.9f;
    public:
        using service_tag = void;

        task<float> get_heading_deg()
        {
            co_return random_float(min_heading_deg, max_heading_deg);
        }
    };

    template <bool use_remote>
    struct configurator
    {
        template<typename local_t, typename remote_t>
        using impl_t = std::conditional_t<use_remote, remote_t, local_t>;

        using hardware_config = hardware_t<
            impl_t<rover_lidar, sense::perception::remote::lidar>,
            impl_t<rover_gnss_receiver, sense::spatial::remote::gnss_receiver>,
            impl_t<rover_magnetometer, sense::spatial::remote::magnetometer>,
            impl_t<motion::propulsion::local::engine_control, motion::propulsion::remote::engine_control>,
            impl_t<motion::articulation::local::servo_array, motion::articulation::remote::servo_array>,
            impl_t<payload::logistics::local::gripper_mech, payload::logistics::remote::gripper_mech>,
            impl_t<motion::articulation::local::lighting, motion::articulation::remote::lighting>
        >;

        using autonomy_config = autonomy_t<
            impl_t<gnc::navigation::local::ekf_fusion, gnc::navigation::remote::ekf_fusion>,
            impl_t<gnc::navigation::local::slam_engine, gnc::navigation::remote::slam_engine>,
            impl_t<rover_global_planner, gnc::guidance::remote::global_planner>,
            // Using custom planner instead of library one
            impl_t<rover_local_planner, gnc::guidance::remote::local_planner>,
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
    using namespace kmx::aether::v0_1;
    using namespace kmx::aether::v0_1::sample::rover;

    std::signal(SIGINT, handle_signal);

    scheduler sched;

    constexpr bool use_remote = false;
    using rover_vehicle = configurator<use_remote>::vehicle;
    rover_vehicle rover;

    const std::vector<waypoint> route{
        waypoint{5.0f, 0.0f},
        waypoint{10.0f, 8.0f},
        waypoint{20.0f, 15.0f}
    };

    rover.ai.path_finder.set_scheduler(sched);
    rover.ai.obstacle_avoid.set_scheduler(sched);

    auto m = rover.mission(std::span<const waypoint>{route}, sched);
    sched.schedule(m.handle);
    sched.run();

    return 0;
}
