#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <TimeoutSerial.h>
#include <models/shared_queue.hpp>
#include <commands/command.hpp>
#include <json.hpp>
#include <string>

namespace scanner
{
    class microcontroller
    {
    public:
        boost::thread thread_controller;
        bool controller_alive;
        shared_queue<std::shared_ptr<command>> commandq;
        TimeoutSerial serial_port;
        std::string devname = "/dev/ttyACM0";
        unsigned int baud = 9600;

        enum flush_type
        {
            flush_in = TCIFLUSH,
            flush_out = TCOFLUSH,
            flush_io = TCIOFLUSH
        };

        microcontroller();
        microcontroller(std::string devname_, unsigned int baud_);
        void invoke_controller(std::shared_ptr<command> comm);
        void serial_open();
        void serial_close();
        void serial_write_string(std::string str);
        bool serial_is_open();
        void serial_flush(flush_type ftype, boost::system::error_code &error);
        std::string serial_readln();
        void serial_set_timeout(int timeout);
        void send_message(std::string msg, nlohmann::json &response, int timeout, bool &err);

        void set_laser(int state, int delay, nlohmann::json &response, int timeout, bool &err);
        void rotate(std::string direction, float angle, nlohmann::json &response, int timeout, bool &err);

        static std::string format(std::string identifier, std::string value);
        static std::string format(std::string identifier, int value);
        static std::string format(std::string identifier, float value);
        static std::string format(std::string identifier);
    };
}

#endif
