#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::rover
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace payload = kmx::aether::v0_1::payload;

    struct hardware
    {
        sense::perception::lidar lidar;
        motion::propulsion::engine_control motors; // Skid steer (Left/Right)
        payload::logistics::gripper_mech claw;
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
