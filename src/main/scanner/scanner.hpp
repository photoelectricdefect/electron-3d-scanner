#ifndef SCANNER_H_
#define SCANNER_H_

#include <napi.h>
#include <iostream>
#include <boost/thread.hpp>
#include <models/shared_queue.hpp>
#include <commands/command.hpp>
#include <camera_calibration.hpp>
#include <scanner_calibration.hpp>
#include <scanconfig.hpp>
#include <microcontroller.hpp>
#include <camera.hpp>
#include <string>
#include <memory>

using camera_ = scanner::camera;
using scanner_calibration_ = scanner::scanner_calibration;

namespace scanner {
    class scanner {
        private:
            void load_config(const std::string& fpath);
            void create_config_file(const std::string& fpath);
        public:            
            boost::thread thread_main;
            
            bool thread_main_alive, scanning, calibrating_scanner, scanner_calibrated;
            
            boost::mutex mutex_calibrating_scanner,mutex_scanning,mutex_scanner_calibrated, mutex_thread_main_alive,
            mtx_calibrated;

            shared_queue<std::shared_ptr<command>> commandq;
            
            camera_ camera;
            
            microcontroller controller;
            
            scanner_calibration_ scanner_calibration;
            
            scanconfig scconfig;

            scanner();
            void init();
            
            bool get_flag_calibrating_scanner();
            void set_flag_calibrating_scanner(bool value);

            bool get_flag_scanner_calibrated();
            void set_flag_scanner_calibrated(bool value);

            bool get_flag_scanning();
            void set_flag_scanning(bool value);

            bool get_flag_thread_main_alive();
            void set_flag_thread_main_alive(bool value);

            int wait_key(int timeout);
            void thread_main_invoke(std::shared_ptr<command> comm);
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