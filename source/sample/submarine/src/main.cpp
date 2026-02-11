#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::submarine
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace assurance = kmx::aether::v0_1::assurance;

    struct hardware
    {
        sense::environment::local::ocean_data depth_sensor; // Pressure
        sense::spatial::local::imu_array imu;
        motion::propulsion::local::thrust_vector thrusters; // 6DOF control
        assurance::fdir::local::fault_detector leak_sensor;
        motion::articulation::local::servo_array ballast_tank;
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
