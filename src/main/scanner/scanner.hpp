#ifndef SCANNER_H_
#define SCANNER_H_

#include <napi.h>
#include <iostream>
#include <boost/thread.hpp>
#include <models/shared_queue.hpp>
#include <commands/command.hpp>
#include <cameracalib.hpp>
#include <models/event.hpp>
#include <string>
#include <memory>

namespace scanner {
    class scanner {
        public:            
            boost::thread threadIO, thread_camera, thread_table;
            boost::mutex mtx_video_alive, mtx_cameracalib;
            bool IOalive, camera_alive, video_alive,
                scanning, calibratingcamera;
            shared_queue<std::shared_ptr<command>> commandq;
            shared_queue<event<int>> camera_inputq, table_inputq;
            cameracalib calib_camera;
            
            scanner();
            //move these to their own command objects
            //--------
            void scan_start();
            void scan_stop();
            void load_point_cloud();
            void setprop();
            //--------

            void invokeIO(std::shared_ptr<command> comm, bool blocking);
            void stremit(std::string e, std::string msg, bool blocking);

            template<typename F>
            void lock(F& fn, boost::mutex& mtx) {
                boost::unique_lock<boost::mutex> lock(mtx);
                fn();
            }
    };
}

#endif