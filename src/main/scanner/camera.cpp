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

void camera::clear_messageq_camera() {
    boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);

    while(camera_messageq.size()>0) {camera_messageq.pop(); }
}

    void camera::post_message_camera(nlohmann::json msg) {
        boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);
        camera_messageq.push(msg);
    }
    
    bool camera::recieve_message_camera(nlohmann::json& msg) {
        boost::unique_lock<boost::mutex> lock(mtx_camera_messageq);
        
        if(camera_messageq.size()>0) {
            msg=camera_messageq.front();
            camera_messageq.pop();

            return true;
        }
        
        return false;
    }
}