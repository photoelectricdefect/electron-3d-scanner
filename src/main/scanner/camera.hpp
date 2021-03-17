#include <json.hpp>
#include <boost/thread.hpp>
#include <cameracalib.hpp>
#include <models/shared_queue.hpp>
#include <string>
#include <queue>

#ifndef CAMERA_H_
#define CAMERA_H_

namespace scanner {
    class camera {
        public:
            boost::thread thread_camera,thread_video;
            boost::mutex mtx_video_alive,mtx_calibrated,mtx_camera_keyq,mtx_camera_messageq;
            bool camera_alive, video_alive, calibrating, calibrated;
            cameracalib calib;
            std::queue<int> camera_keyq;
            std::queue<nlohmann::json> camera_messageq;

            camera();
            void clear_key_camera();
            void set_key_camera(int keycode);
            int get_key_camera();
            void clear_messageq_camera();
            void post_message_camera(nlohmann::json msg);
            bool recieve_message_camera(nlohmann::json& msg);
    };
}

#endif
