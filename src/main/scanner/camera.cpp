#include <camera.hpp>

namespace scanner {
    camera::camera() {
        //TODO: load calib from file
        calib.load();
    }
}