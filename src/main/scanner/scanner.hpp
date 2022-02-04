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
            boost::mutex mtx_calibrated,mtx_scanning;
            shared_queue<std::shared_ptr<command>> commandq;
            camera_ camera;
            microcontroller controller;
            scannercalib sccalib;
            scanconfig scconfig;


            scanner();
            //move these to their own command objects
            //--------
            void load_point_cloud();
            //--------

            int wait_key(int timeout);
            void invokeIO(std::shared_ptr<command> comm);
            void stremit(std::string e, std::string msg, bool blocking);
            void imemit(std::string e, uint8_t* imbase64, size_t len, bool blocking);
            void imemit(std::string e, uint8_t* imbase64, std::string msg, size_t len, bool blocking);

            template<typename F>
            void lock(F& fn, boost::mutex& mtx) {
                boost::unique_lock<boost::mutex> lock(mtx);
                fn();
            }
    };
}

#endif