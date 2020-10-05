#include <json.hpp>
#include <boost/thread.hpp>
#include <cameracalib.hpp>
#include <models/shared_queue.hpp>
#include <string>

#ifndef CAMERA_H_
#define CAMERA_H_

namespace scanner {
    class camera {
        public:
            boost::thread thread_camera;
            boost::shared_mutex mtx_video_alive;
            bool thread_alive, video_alive, calibrating, calibrated;
            shared_queue<nlohmann::json> inputq;
            cameracalib calib;

            camera();
    };
}

#endif
