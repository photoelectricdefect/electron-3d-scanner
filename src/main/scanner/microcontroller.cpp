#include <microcontroller.hpp>

namespace scanner {

    microcontroller::microcontroller() {}
    microcontroller::microcontroller(std::string devname_, unsigned int baud_) : devname(devname_), baud(baud_)  {}

    void microcontroller::serial_open() {
        serial_port.open(devname, baud);
    }

    void microcontroller::serial_close() {
        serial_port.close();
    }

    std::string microcontroller::serial_readln() {
        return serial_port.readStringUntil();
    }

    void microcontroller::serial_writeln(std::string str) {
        serial_port.writeString(str + "\n");
    }

    void microcontroller::serial_set_timeout(int timeout) {
        serial_port.setTimeout(boost::posix_time::milliseconds(timeout));
    }    

    bool microcontroller::serial_is_open() {
        return serial_port.isOpen();
    }  

    std::string microcontroller::format(std::string identifier, int value) {
        std::ostringstream s;
        s << identifier << ":" << value << ";";
        return s.str();
    }            

    std::string microcontroller::format(std::string identifier, float value) {
        std::ostringstream s;
        s << identifier << ":" << value << ";";
        return s.str();
    }       

    std::string microcontroller::format(std::string identifier) {
        return identifier + ";";
    }  
}