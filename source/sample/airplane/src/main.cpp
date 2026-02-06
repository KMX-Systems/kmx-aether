#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::airplane
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;

    struct hardware
    {
        sense::environment::air_data pitot_tube;    // Airspeed
        sense::spatial::gnss_receiver gps;          // Position
        motion::articulation::servo_array surfaces; // Aileron, Elevator, Rudder, Throttle
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
