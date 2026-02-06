#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::vessel
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace payload = kmx::aether::v0_1::payload;

    struct hardware
    {
        sense::perception::radar marine_radar;
        sense::spatial::gnss_receiver gps;
        motion::articulation::servo_array rudders;
        motion::propulsion::engine_control diesel_jets;
        telematics::link::satcom_modem satellite;
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
