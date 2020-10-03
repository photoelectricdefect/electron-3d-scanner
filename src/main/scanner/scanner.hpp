#ifndef SCANNER_H_
#define SCANNER_H_

#include <napi.h>
#include <iostream>
#include <boost/thread.hpp>
#include <models/shared_queue.hpp>
#include <commands/command.hpp>
#include <cameracalib.hpp>
#include <scannercalib.hpp>
#include <scanconfig.hpp>
#include <microcontroller.hpp>
#include <camera.hpp>
#include <string>
#include <memory>

using camera_ = scanner::camera;

namespace scanner {
    class scanner {
        public:            
            boost::thread threadIO;
            bool IOalive, scanning, calibrating, calibrated;
            shared_queue<std::shared_ptr<command>> commandq;
            camera_ camera;
            microcontroller controller;
            scannercalib calib;
            scanconfig scconf;

            scanner();
            //move these to their own command objects
            //--------
            void load_point_cloud();
            //--------

            void invokeIO(std::shared_ptr<command> comm, bool blocking);
            void stremit(std::string e, std::string msg, bool blocking);

            template<typename F>
            void lock(F& fn, boost::shared_mutex& mtx, bool shared) {
                if(shared) boost::shared_lock<boost::shared_mutex> lock(mtx);  
                else boost::unique_lock<boost::shared_mutex> lock(mtx);
                fn();
            }
    };
}

#endif