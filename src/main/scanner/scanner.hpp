#include <napi.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <models/threadsafe_queue.hpp>
#include <models/command.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace scanner {
            const std::string EV_ERROR = "error",
                EV_STATUS = "status",
                EV_IMUPDATE = "imupdate",
                EV_VIDEOSTART = "videostart",
                EV_VIDEOSTOP = "videostop",
                EV_IOSTART = "iostart",
                EV_IOSTOP = "iostop";

    class scanner {
        private:
            bool _scanning;
            bool _calibrating;

        public:
            boost::thread threadIO, thread_video;
            threadsafe_queue<command> commandq;
            
            scanner();
            //move these to their own command objects
            //--------
            void video_start();
            void video_stop();
            void scan_start();
            void scan_stop();
            void load_point_cloud();
            void setprop();
            void iostart();
            //--------

            void send_command(command comm);
            void try_send_command(command comm);
            bool is_scanning();
            bool is_calibrating();
    };

    void stremit(std::string e, std::string msg, bool blocking);
    void add_listener(const Napi::CallbackInfo& info);
    void remove_listener(const Napi::CallbackInfo& info);
    void send_command(const Napi::CallbackInfo& info);
}