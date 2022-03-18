#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <boost/thread.hpp>
#include <models/shared_queue.hpp>
#include <commands/command.hpp>
#include <json.hpp>
#include <string>
#include <serial/serial.h>

namespace scanner
{
    class microcontroller
    {
    private:
        const int _DELAY_PORT_OPEN_MS=1000;     

        serial::Serial _serial_port;
        std::string _device_name;
        unsigned int _baud_rate;
        bool _is_serial_port_configured;
    public:
        boost::mutex mutex_thread_controller;
        boost::thread thread_controller;
        bool thread_controller_alive;
        shared_queue<std::shared_ptr<command>> commandq;

        microcontroller();

        void invoke_controller(std::shared_ptr<command> comm);

        bool get_flag_thread_controller_alive();
        void set_flag_thread_controller_alive(bool value);

        void configure_serial_port(const std::string& device_name, const unsigned int baud_rate);
        void send_message(std::string msg, nlohmann::json &response, int timeout, bool &err);
        void set_laser(int state, int delay, nlohmann::json &response, int timeout, bool &err);
        void rotate(std::string direction, float angle,int delay, nlohmann::json &response, int timeout, bool &err);
    };
}

#endif
