#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <boost/thread.hpp>
#include <TimeoutSerial.h>
#include <json.hpp>
#include <string>

namespace scanner {
    class microcontroller {
        public:
            TimeoutSerial serial_port;
            std::string devname;
            unsigned int baud;

            microcontroller();
            microcontroller(std::string devname_, unsigned int baud_);
            void serial_open();
            void serial_close();
            void serial_writeln(std::string str);
            bool serial_is_open();
            std::string serial_readln();
            void serial_set_timeout(int timeout);
            static std::string format(std::string identifier, int value);            
            static std::string format(std::string identifier);            
    };
}

#endif
