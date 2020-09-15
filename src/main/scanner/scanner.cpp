#include <scanner.hpp>
#include <opencv2/videoio.hpp>
#include <base64.h>
#include <models/config.hpp>
#include <models/cv_helpers.hpp>
#include <models/math_helpers.hpp>

//BIG TODO: rework to use scanner object as context to command objects
namespace scanner {
            // void list_handlers() {

            //     std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.begin();
    
            //     while(it != ev_handlers.end()) {
            //         std::cout << "key: " << it->first << std::endl;
            //         it++;
            //     }  

            //     std::cout << "------------------------" << std::endl;
            // }
            scanner::scanner() {};

            void scanner::video_start() {
                auto fn = [](){
                try {
            cv::VideoCapture cap;
	        cap.open(0);

	        if (!cap.isOpened())
	        {
		        std::cerr << "error opening camera" << std::endl;
		        return;
	        }

	        std::cout << "space = capture image" << std::endl;

	        cv::Mat frame;
	        bool running = true;

            while(running) {
		        boost::this_thread::interruption_point(); 

                char c = cv::waitKey((int)(1000 / FPS_60));
		        cap.read(frame);

		        if (frame.empty())
		        {
			        std::cerr << "empty frame grabbed" << std::endl;
			        continue;
		        }
        
                stremit(EV_IMUPDATE, cv_helpers::mat2base64str(frame), true);
            }

            }
            catch (boost::thread_interrupted&) {}
        };
    
        thread_video = boost::thread{fn};
        stremit(EV_VIDEOSTART, "", true);
    }

            void scanner::video_stop() {
                thread_video.interrupt();
                thread_video.join();
                stremit(EV_VIDEOSTOP, "", true);
            };

            void scanner::scan_start() {};
            void scanner::scan_stop() {};
            void scanner::load_point_cloud() {};
            void scanner::setprop() {};

            void scanner::iostart() {
                auto fn = [this]() {
                    commandq.clear();
                    bool running = true;

                    while(running) {
                        command comm = commandq.dequeue();
                        //commandq.lockq();

                        if(comm.code == COMM_VIDEOSTART) {
                            video_start();
                        }
                        else if(comm.code == COMM_VIDEOSTOP) {
                            video_stop();
                        }
                        else if(comm.code == COMM_IOSTOP) {
                            video_stop();
                            running = false;
                        }

                        //commandq.unlockq();
                    }
                
                    stremit(EV_IOSTOP, "", true);
                };

                threadIO = boost::thread{fn};
                stremit(EV_IOSTART, "", true);
            }


            void scanner::send_command(command comm) { commandq.enqueue(comm); }
            void scanner::try_send_command(command comm) { commandq.try_enqueue(comm); }
            bool scanner::is_scanning() { return false; }
            bool scanner::is_calibrating() { return false; }

//TODO: turn into object, sort out public/private variables etc...
//API----------------------------------

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
    sc.commandq.try_enqueue(command(j.get<jcommand>()));
}

Napi::ThreadSafeFunction stremitTSFN;
void _stremit(const Napi::CallbackInfo& info) {
    Napi::String e = info[0].As<Napi::String>();
    Napi::String msg = info[1].As<Napi::String>();
    std::map<std::string, Napi::FunctionReference>::iterator it = ev_handlers.find(e.Utf8Value());
    
    if(it != ev_handlers.end()) it->second.Value().Call({msg});
};
void stremit(std::string e, std::string msg, bool blocking) {    
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
