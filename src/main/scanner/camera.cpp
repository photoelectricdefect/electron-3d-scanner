#include <camera.hpp>

#include <iostream>

namespace scanner {
    camera::camera() {
        calibrated=calib.load("cameracalib.json");
    }

void camera::clear_key_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);
    std::queue<int> empty;
    std::swap(camera_keyq,empty);
}

void camera::set_key_camera(int keycode) {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);

    if(camera_keyq.size()>0) {
        std::queue<int> empty;
        std::swap(camera_keyq,empty);
    }

    camera_keyq.push(keycode);
}

int camera::get_key_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_keyq);

    if(camera_keyq.size()<=0) return -1;

    int keycode = camera_keyq.front();
    camera_keyq.pop();

    return keycode;
}

}