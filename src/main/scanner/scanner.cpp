#include <scanner.hpp>
#include <base64.h>
#include <helpers/cv_helpers.hpp>
#include <helpers/math_helpers.hpp>
#include <commands/command.hpp>
#include <commands/command_iostart.hpp>
#include <commands/command_iostop.hpp>
#include <commands/command_videostart.hpp>
#include <commands/command_videostop.hpp>
#include <commands/command_cameracalibstart.hpp>
#include <commands/command_cameracalibstop.hpp>
#include <commands/command_scannercalibstart.hpp>
#include <commands/command_scannercalibstop.hpp>
#include <commands/command_scanstart.hpp>
#include <commands/command_scanstop.hpp>
#include <commands/command_lambda.hpp>
#include <boost/thread.hpp>
#include <flags.hpp>

namespace scanner {  
            scanner::scanner() {
                calib.load();
                scconf.load();
            }

            void scanner::load_point_cloud() {}

            void scanner::invokeIO(std::shared_ptr<command> comm, bool blocking) {  
                if(comm->code != COMM_IOSTART && !IOalive) return;
                if(comm->code == COMM_IOSTART && IOalive) return;

                if(comm->code == COMM_IOSTART || comm->code == COMM_IOSTOP) comm->execute(comm);
                else {
                    if(blocking) commandq.enqueue(comm);
                    else commandq.try_enqueue(comm);
                } 
            }

scanner sc;
std::map<std::string, Napi::FunctionReference> ev_handlers;        

void add_listener(const Napi::CallbackInfo& info) {
    Napi::Function fn = info[1].As<Napi::Function>();
    std::string e = info[0].As<Napi::String>().Utf8Value() ;
    auto it = ev_handlers.find(e);
    
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
            sc.invokeIO(std::shared_ptr<command>(new command_iostart(sc, jcomm)), true);
            break;
        case COMM_IOSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_iostop(sc, jcomm)), true);        
            break;
        case COMM_VIDEOSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_videostart(sc, jcomm)), true);
            break;
        case COMM_VIDEOSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_videostop(sc, jcomm)), true);
            break;
        case COMM_CAMERACALIBSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_cameracalibstart(sc, jcomm)), true);
            break;
        case COMM_CAMERACALIBSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_cameracalibstop(sc, jcomm)), true);
            break;            
        case COMM_SCANSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_scanstart(sc, jcomm)), true);
            break; 
        case COMM_SCANSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_scanstop(sc, jcomm)), true);
            break; 
    }
}

void keyboard_input(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    int code = j["code"].get<int>();

    auto comm = [j]() {
        sc.camera.inputq.enqueue(j);
    };

    sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, code, comm)), true);
}

Napi::ThreadSafeFunction stremitTSFN;
// void _stremit(const Napi::CallbackInfo& info) {
//     Napi::String e = info[0].As<Napi::String>();
//     Napi::String msg = info[1].As<Napi::String>();
//     auto it = ev_handlers.find(e.Utf8Value());
    
//     if(it != ev_handlers.end()) it->second.Value().Call({msg});
// };
//FIX: after a while data seems to be buffered when sending large strings rapidly, as the RAM usage starts to increase; no idea why
void scanner::stremit(std::string e, std::string msg, bool blocking) {    
    auto callback = [e, msg]( Napi::Env env, Napi::Function jscb) {
      //jscb.Call( { Napi::String::New(env, e), Napi::String::New(env, msg) });

        // Napi::String e = info[0].As<Napi::String>();
        // Napi::String msg = info[1].As<Napi::String>();
        auto it = ev_handlers.find(e);
    
        if(it != ev_handlers.end()) it->second.Value().Call({Napi::String::New(env, msg)});
    };

    if(blocking) stremitTSFN.BlockingCall(callback);
    else stremitTSFN.NonBlockingCall(callback);
}

Napi::ThreadSafeFunction imemitTSFN;
void scanner::imemit(std::string e, std::shared_ptr<std::string> imbase64, bool blocking) {    
    auto callback = [e, imbase64]( Napi::Env env, Napi::Function jscb) {
        auto it = ev_handlers.find(e);
    
        if(it != ev_handlers.end()) it->second.Value().Call({Napi::String::New(env, *imbase64)});
    };

    if(blocking) imemitTSFN.BlockingCall(callback);
    else imemitTSFN.NonBlockingCall(callback);
}

void setprop(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    int code = j["code"].get<int>();
    int prop = j["prop"].get<int>();

    switch(prop) {
        case PROP_VIDEOALIVE:
            bool val = j["value"].get<bool>();

            auto comm = [val]() {
                auto setval = [val]() {
                    sc.camera.video_alive = val;
                };

                sc.lock(setval, sc.camera.mtx_video_alive, false);
                nlohmann::json j;
                j["prop"] = PROP_VIDEOALIVE;
                j["value"] = val;
                std::cout << j.dump() << std::endl;
                sc.stremit(EV_PROPCHANGED, j.dump(), true);    
            };

            sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, code, comm)), true);
            break;
    }
}

Napi::Value getprop(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    int prop = info[0].As<Napi::Number>().Int32Value();
    
    struct tsfn_ctx {
        tsfn_ctx(Napi::Env env) : deferred(Napi::Promise::Deferred::New(env)) {};

        Napi::Promise::Deferred deferred;
        Napi::ThreadSafeFunction tsfn;
    };

    std::shared_ptr<tsfn_ctx> ctx(new tsfn_ctx(env));
    ctx->tsfn = Napi::ThreadSafeFunction::New(env,  
    Napi::Function::New(env, [](const Napi::CallbackInfo& info){ return info.Env().Undefined(); } ), 
    "getprop", 0, 1); 

    auto comm = [ctx, prop]() {
        auto getprop = [ctx, prop](Napi::Env env, Napi::Function jscb) {
            switch(prop) {
                case PROP_VIDEOALIVE:
                    auto getval = [ctx, &prop]() {
                        ctx->deferred.Resolve(Napi::Boolean::New(ctx->deferred.Env(), sc.camera.video_alive));
                    };

                    sc.lock(getval, sc.camera.mtx_video_alive, true);       
                    break;
            }
        };

        ctx->tsfn.NonBlockingCall(getprop);
        ctx->tsfn.Release();
    };

    sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, COMM_GETPROP, comm)), true);

    return ctx->deferred.Promise();
}

Napi::Object init(Napi::Env env, Napi::Object exports) {
    exports.Set(Napi::String::New(env, "addListener"), 
                    Napi::Function::New(env, add_listener));
    exports.Set(Napi::String::New(env, "removeListener"), 
                    Napi::Function::New(env, remove_listener));
    exports.Set(Napi::String::New(env, "sendCommand"), 
                    Napi::Function::New(env, send_command));
    exports.Set(Napi::String::New(env, "keyboardInput"), 
                    Napi::Function::New(env, keyboard_input));
    exports.Set(Napi::String::New(env, "setProp"), 
                    Napi::Function::New(env, setprop));
    // stremitTSFN = Napi::ThreadSafeFunction::New(env, Napi::Function::New<_stremit>(env), "stremit", 0, 1);
    stremitTSFN = Napi::ThreadSafeFunction::New(env,  Napi::Function::New(env, [](const Napi::CallbackInfo& info){ return info.Env().Undefined(); } ), "stremit", 0, 1);
    imemitTSFN = Napi::ThreadSafeFunction::New(env,  Napi::Function::New(env, [](const Napi::CallbackInfo& info){ return info.Env().Undefined(); } ), "imemit", 5, 1);

    return exports;
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, init)
}
