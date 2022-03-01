#include <json.hpp>
#include <boost/thread.hpp>
#include <camera_calibration.hpp>
#include <models/shared_queue.hpp>
#include <string>
#include <queue>

#ifdef __linux__ 
    #include <linux/videodev2.h>
#elif _WIN32
    // TODO
#endif


#ifndef CAMERA_H_
#define CAMERA_H_

using camera_calibration_ = scanner::camera_calibration;

namespace scanner {
    class camera {
        private:
            #ifdef __linux__ 
            static std::vector<std::pair<int,v4l2_capability>> get_v4l2_capabilities();
            #elif
            //TODO
            #endif

        public:
            typedef struct camera_info {
                int id;
                std::string name;                
            } camera_info;

            boost::thread thread_camera, thread_video;
            
            boost::mutex mutex_thread_video_alive, mutex_display_video,
                        mutex_video_capture, mutex_thread_camera_alive,
                        mutex_calibrating_camera,mutex_message_thread_camera,
                        mutex_camera_calibrated;
            
            boost::condition_variable condition_display_video, condition_message_thread_camera;

            bool calibrating_camera, camera_calibrated, thread_video_alive, display_video, thread_camera_alive;

            camera_calibration_ camera_calibration;
            
            cv::VideoCapture video_capture;

            nlohmann::json* message_thread_camera;
            
            camera();

            bool get_flag_display_video();
            void set_flag_display_video(bool value);

            bool get_flag_thread_video_alive();
            void set_flag_thread_video_alive(bool value);

            bool get_flag_thread_camera_alive();
            void set_flag_thread_camera_alive(bool value);

            bool get_flag_calibrating_camera();
            void set_flag_calibrating_camera(bool value);

            bool get_flag_camera_calibrated();
            void set_flag_camera_calibrated(bool value);

            void clear_message_thread_camera();
            void set_message_thread_camera(const nlohmann::json& message);
            void get_message_thread_camera(nlohmann::json& message);
            void try_get_message_thread_camera(nlohmann::json* message);

            static std::vector<camera_info> get_camera_list();

            #ifdef __linux__ 
            static int get_videoid(std::string camera_name);            
            #elif
            //TODO
            #endif

    };
}

#endif
