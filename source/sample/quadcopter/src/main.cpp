#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::quadcopter
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace telematics = kmx::aether::v0_1::telematics;

    struct hardware
    {
        sense::spatial::local::imu_array imu;             // Gyro/Accel
        motion::propulsion::local::esc_driver motors;     // 4x ESCs
        motion::power::local::bms_manager battery;        // Smart Battery
        telematics::link::local::rcem_receiver rc_remote; // Pilot Input
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
