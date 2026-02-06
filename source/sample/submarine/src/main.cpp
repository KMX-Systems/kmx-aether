#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::submarine
{
    struct hardware
    {
        sense::environment::ocean_data depth_sensor; // Pressure
        sense::spatial::imu_array imu;
        motion::propulsion::thrust_vector thrusters; // 6DOF control
        assurance::fdir::fault_detector leak_sensor;
        motion::articulation::servo_array ballast_tank;
    };
}

int main() noexcept
{
    try
    {
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
