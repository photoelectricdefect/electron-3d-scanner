#ifndef SCANNER_H_
#define SCANNER_H_

#include <napi.h>
#include <iostream>
#include <boost/thread.hpp>
#include <models/threadsafe_queue.hpp>
#include <commands/command.hpp>
#include <string>
#include <memory>

namespace scanner {
    class scanner {
        public:            
            boost::thread threadIO, thread_video;
            threadsafe_queue<std::shared_ptr<command>> commandq;
            
            scanner();
            //move these to their own command objects
            //--------
            void scan_start();
            void scan_stop();
            void load_point_cloud();
            void setprop();
            //--------

            void send_command(std::shared_ptr<command> comm, bool blocking);
            void stremit(std::string e, std::string msg, bool blocking);

            void set_IOalive(bool val);
            void set_video_alive(bool val);            
            void set_scanning(bool val);
            void set_calibrating(bool val);

            bool get_IOalive();
            bool get_video_alive();            
            bool get_scanning();
            bool get_calibrating();
    };
}

#endif