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

            boost::thread thread_camera, thread_video_capture, thread_video_open;
            
            boost::mutex mutex_thread_video_open_alive, mutex_thread_video_capture_alive,
                        mutex_video_capture, mutex_thread_camera_alive,
                        mutex_calibrating_camera,mutex_message_thread_camera,
                        mutex_camera_calibrated,/*mutex_selected_camera_info,*/
                        mutex_video_open;

            boost::condition_variable condition_message_thread_camera,
            /*condition_selected_camera_info,*/condition_video_open;

            bool calibrating_camera, camera_calibrated, video_open, 
            thread_video_open_alive, thread_video_capture_alive,
            thread_camera_alive;

            camera_calibration_ camera_calibration;
            
            cv::VideoCapture video_capture;

            std::unique_ptr<nlohmann::json> message_thread_camera;
            
            std::unique_ptr<camera_info> selected_camera_info;

            camera();

            // bool get_flag_display_video();
            // void set_flag_display_video(bool value);

            // bool notify_video_open();
            void notify_video_closed();
            
            void set_selected_camera_info(const camera_info& cam_info);
            void get_selected_camera_info(camera_info& cam_info);

            bool get_flag_thread_video_open_alive();
            void set_flag_thread_video_open_alive(bool value);

            bool get_flag_thread_video_capture_alive();
            void set_flag_thread_video_capture_alive(bool value);

            bool get_flag_thread_camera_alive();
            void set_flag_thread_camera_alive(bool value);

            bool get_flag_calibrating_camera();
            void set_flag_calibrating_camera(bool value);

            bool get_flag_camera_calibrated();
            void set_flag_camera_calibrated(bool value);

            void clear_message_thread_camera();
            void set_message_thread_camera(const nlohmann::json& message);
            void get_message_thread_camera(nlohmann::json& message);
            bool try_get_message_thread_camera(nlohmann::json& message);

            static std::vector<camera_info> get_camera_info_list();
            
            #ifdef __linux__ 
            static int get_videoid(std::string camera_name);            
            #elif
            //TODO
            #endif

    };
}

#endif
