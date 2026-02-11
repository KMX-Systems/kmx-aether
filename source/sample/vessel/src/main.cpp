#include <iostream>
#include <kmx/aether/aether.hpp>

namespace kmx::aether::v0_1::sample::vessel
{
    namespace sense = kmx::aether::v0_1::sense;
    namespace motion = kmx::aether::v0_1::motion;
    namespace payload = kmx::aether::v0_1::payload;
    namespace telematics = kmx::aether::v0_1::telematics;

    struct hardware
    {
        sense::perception::local::radar marine_radar;
        sense::spatial::local::gnss_receiver gps;
        motion::articulation::local::servo_array rudders;
        motion::propulsion::local::engine_control diesel_jets;
        telematics::link::local::satcom_modem satellite;
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
