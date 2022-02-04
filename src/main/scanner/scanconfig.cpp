#include <scanconfig.hpp>

namespace scanner
{
    scanconfig::scanconfig(){};

    // TODO: read configuration and saved data from files
    void scanconfig::load()
    {
        rotation_direction = ROTDIR_CLOCKWISE;
        rotation_resolution = ROTATION_RESOLUTION_FULLSTEP;
    }
}
