#include <microcontroller.hpp>
#include <iostream>

namespace scanner {

    microcontroller::microcontroller() {}
    microcontroller::microcontroller(std::string devname_, unsigned int baud_) : devname(devname_), baud(baud_)  {}

    void microcontroller::invoke_controller(std::shared_ptr<command> comm) {
        boost::unique_lock<boost::mutex> lock(commandq.mtx);

        if(commandq.q.size()>1 || !controller_alive) return;

        commandq.q.push(comm);
        commandq.condition.notify_one();
    }

    void microcontroller::serial_open() {
        serial_port.open(devname, baud);
    }

    void microcontroller::serial_close() {
        serial_port.close();
    }

    std::string microcontroller::serial_readln() {
        return serial_port.readStringUntil("\n");
    }

    void microcontroller::serial_write_string(std::string str) {
        serial_port.writeString(str);
    }

    void microcontroller::serial_set_timeout(int timeoutms) {
        serial_port.setTimeout(boost::posix_time::milliseconds(timeoutms));
    }    

    bool microcontroller::serial_is_open() {
        return serial_port.isOpen();
    }  

    //https://stackoverflow.com/questions/22581315/how-to-discard-data-as-it-is-sent-with-boostasio
    //boost has no wrapper for flushing so this should only work on linux, FIX
void microcontroller::serial_flush(flush_type ftype,boost::system::error_code& error)
{
  if (0 == ::tcflush(serial_port.get_port()->lowest_layer().native_handle(), ftype))
  {
    error = boost::system::error_code();
  }
  else
  {
    error = boost::system::error_code(errno,
        boost::asio::error::get_system_category());
  }
    }

        void microcontroller::send_message(std::string msg,nlohmann::json& response,int timeout,bool& err) {
                    try {
                        if (!serial_is_open()) {
                            serial_open();
                            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
                        }
                            
                        // std::cout<<"sent to controler"<<std::endl;

                        boost::system::error_code ferr;
                        serial_flush(microcontroller::flush_io, ferr);

                        if (ferr)
                            BOOST_THROW_EXCEPTION(std::runtime_error(ferr.message()));

                        serial_write_string(msg + "\r");
                        serial_set_timeout(timeout);
                        response=serial_readln();
                    }
                    catch (boost::system::system_error& e) {
                        err = true;
                        std::cerr << boost::diagnostic_information(e);
                    }
                    catch (timeout_exception& e) {
                        err = true;
                        std::cerr << boost::diagnostic_information(e);
                    }
                    catch (std::exception& e) {
                        err = true;
                        std::cerr << boost::diagnostic_information(e);
                    }
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