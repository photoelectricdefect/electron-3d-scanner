#include <microcontroller.hpp>
#include <iostream>

namespace scanner
{
    microcontroller::microcontroller() {}

    void microcontroller::invoke_controller(std::shared_ptr<command> comm)
    {
        boost::unique_lock<boost::mutex> lock(commandq.mtx);

        if (commandq.q.size() > 1 || !get_flag_thread_controller_alive())
            return;

        commandq.q.push(comm);
        commandq.condition.notify_one();
    }


    void microcontroller::configure_serial_port(const std::string& device_name, const unsigned int baud_rate) {
        _device_name=device_name;
        _baud_rate=baud_rate;
        _serial_port.setPort(_device_name);
        _serial_port.setBaudrate(_baud_rate);
        _is_serial_port_configured=true;
    }

    void microcontroller::send_message(std::string msg, nlohmann::json &response, int timeoutms, bool &err)
    {
        if(!_is_serial_port_configured)
        {
            return;
        }

        try
        {
            if (!_serial_port.isOpen())
            {
                _serial_port.open();
                boost::this_thread::sleep_for(boost::chrono::milliseconds(_DELAY_PORT_OPEN_MS));
            }

            _serial_port.flush();
            auto timeout_write=serial::Timeout::simpleTimeout(timeoutms);
            _serial_port.setTimeout(timeout_write);
            auto t0 = boost::posix_time::microsec_clock::local_time();
            _serial_port.write(msg + "\r");
            auto t1 = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration dt=t1-t0;
            int dt_ms=dt.total_milliseconds();

            if(dt_ms>=timeoutms) {
                err=true;
                return;
            }

            auto timeout_read=serial::Timeout::simpleTimeout(timeoutms-dt_ms);
            _serial_port.setTimeout(timeout_read);
            std::string string_response;
            _serial_port.readline(string_response);
            response=string_response;
        }
        catch (const serial::IOException &e)
        {
            err = true;
            std::cerr << e.what();
        }
        catch (const serial::SerialException &e)
        {
            err = true;
            std::cerr << e.what();
        }
        catch (serial::PortNotOpenedException &e)
        {
            err = true;
            std::cerr << e.what();
        }
    }

    void microcontroller::set_laser(int state, int delay, nlohmann::json &response, int timeout, bool &err)
    {        
        nlohmann::json message;
        message["name"]="laser";
        message["state"]=state;
        message["delay"]=delay;
        send_message(message.dump(), response, timeout, err);
    }

    void microcontroller::rotate(std::string direction, float steps, int delay, nlohmann::json &response, int timeout, bool &err)
    {
        nlohmann::json message;
        message["name"]="rotate";
        message["direction"]=direction;
        message["steps"]=steps;
        message["delay"]=delay;
        send_message(message.dump(), response, timeout, err);
    }

    bool microcontroller::get_flag_thread_controller_alive() {
        boost::unique_lock<boost::mutex> lock(mutex_thread_controller);
        return thread_controller_alive;
    }

    void microcontroller::set_flag_thread_controller_alive(bool value) {
        boost::unique_lock<boost::mutex> lock(mutex_thread_controller);
        thread_controller_alive=value;
    }

}