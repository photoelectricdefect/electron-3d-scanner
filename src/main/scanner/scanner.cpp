#include <scanner.hpp>
#include <base64.h>
#include <models/config.hpp>
#include <helpers/cv_helpers.hpp>
#include <helpers/math_helpers.hpp>
#include <commands/command.hpp>
#include <commands/command_iostart.hpp>
#include <commands/command_iostop.hpp>
#include <commands/command_videostart.hpp>
#include <commands/command_videostop.hpp>
#include <boost/thread.hpp>
#include <flags.hpp>

//BIG TODO: rework to use scanner object as context to command objects
namespace scanner {  
            bool IOalive, video_alive,
                scanning, calibrating;
            
            boost::mutex mtx_scanning, mtx_calibrating,
                        mtx_video_alive, mtx_IOalive;

            scanner::scanner() {}

            void scanner::scan_start() {}
            void scanner::scan_stop() {}
            void scanner::load_point_cloud() {}
            void scanner::setprop() {}

            void scanner::send_command(std::shared_ptr<command> comm, bool blocking) {  
                if(comm->code != COMM_IOSTART && !IOalive) return;
                if(comm->code == COMM_IOSTART && IOalive) return;

                if(comm->code == COMM_IOSTART || comm->code == COMM_IOSTOP) comm->execute(comm);
                else {
                    if(blocking) commandq.enqueue(comm);
                    else commandq.try_enqueue(comm);
                } 
            }
 
            bool scanner::get_scanning() {
                boost::unique_lock<boost::mutex> lock(mtx_scanning);
                return scanning;
            }

            bool scanner::get_calibrating() {
                boost::unique_lock<boost::mutex> lock(mtx_calibrating);
                return calibrating;
            }

            bool scanner::get_IOalive() {
                boost::unique_lock<boost::mutex> lock(mtx_IOalive);
                return IOalive;
            }

            bool scanner::get_video_alive() {
                boost::unique_lock<boost::mutex> lock(mtx_video_alive);
                return video_alive;
            }

            void scanner::set_scanning(bool val) {
                boost::unique_lock<boost::mutex> lock(mtx_scanning);
                scanning = val;
            }

            void scanner::set_calibrating(bool val) {
                boost::unique_lock<boost::mutex> lock(mtx_calibrating);
                calibrating = val;
            }

            void scanner::set_IOalive(bool val) {
                boost::unique_lock<boost::mutex> lock(mtx_IOalive);
                IOalive = val;
            }

            void scanner::set_video_alive(bool val) {
                boost::unique_lock<boost::mutex> lock(mtx_video_alive);
                video_alive = val;
            }
scanner sc;
std::map<std::string, Napi::FunctionReference> ev_handlers;        

void add_listener(const Napi::CallbackInfo& info) {
    Napi::Function fn = info[1].As<Napi::Function>();
    std::string e = info[0].As<Napi::String>().Utf8Value() ;
    std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.find(e);
    
    if(it == ev_handlers.end()) ev_handlers.insert(std::pair<std::string, Napi::FunctionReference>(e, Napi::Persistent(fn)));
    else it->second = Napi::Persistent(fn);
}

void remove_listener(const Napi::CallbackInfo& info) {
    std::string e = info[0].As<Napi::String>().Utf8Value() ;
    ev_handlers.erase(e);
}

void send_command(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    jcommand jcomm = j.get<jcommand>();

    switch(jcomm.code) {
        case COMM_IOSTART:
            sc.send_command(std::shared_ptr<command>(new command_iostart(sc, jcomm)), true);
            break;
        case COMM_IOSTOP:
            sc.send_command(std::shared_ptr<command>(new command_iostop(sc, jcomm)), true);        
            break;
        case COMM_VIDEOSTART:
            sc.send_command(std::shared_ptr<command>(new command_videostart(sc, jcomm), [=](command* comm) {std::cout << "videostart command destroyed" << std::endl;}), true);
            break;
        case COMM_VIDEOSTOP:
            sc.send_command(std::shared_ptr<command>(new command_videostop(sc, jcomm)), true);
            break;
    }
}

Napi::ThreadSafeFunction stremitTSFN;
void _stremit(const Napi::CallbackInfo& info) {
    Napi::String e = info[0].As<Napi::String>();
    Napi::String msg = info[1].As<Napi::String>();
    std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.find(e.Utf8Value());
    
    if(it != ev_handlers.end()) it->second.Value().Call({msg});
};
void scanner::stremit(std::string e, std::string msg, bool blocking) {    
    auto callback = [e, msg]( Napi::Env env, Napi::Function jscb) {
      jscb.Call( { Napi::String::New(env, e), Napi::String::New(env, msg) });
    };

    if(blocking) stremitTSFN.BlockingCall(callback);
    else stremitTSFN.NonBlockingCall(callback);
}

Napi::Object init(Napi::Env env, Napi::Object exports) {
    exports.Set(Napi::String::New(env, "addListener"), 
                    Napi::Function::New(env, add_listener));
    exports.Set(Napi::String::New(env, "removeListener"), 
                    Napi::Function::New(env, remove_listener));
    exports.Set(Napi::String::New(env, "sendCommand"), 
                    Napi::Function::New(env, send_command));
    stremitTSFN = Napi::ThreadSafeFunction::New(env, Napi::Function::New<_stremit>(env), "stremit", 0, 1);

    return exports;
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, init)
}
