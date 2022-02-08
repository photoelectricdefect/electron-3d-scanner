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
#include <commands/command_microcontrollerstart.hpp>
#include <commands/command_microcontrollerstop.hpp>
#include <commands/command_rotate.hpp>
#include <commands/command_laserset.hpp>


#include <boost/thread.hpp>
#include <globals.hpp>

namespace scanner {  
scanner sc;
std::map<std::string, Napi::FunctionReference> ev_handlers;        

scanner::scanner() {
    //calibrated=calib.load("scannercalib.json");
    scconfig.load();
    bool scanner_calibrated=sccalib.load("scanner_calib.json");
    bool camera_calibrated=camera.calib.load("cameracalib.json");
    calibrated=scanner_calibrated;
    camera.calibrated=camera_calibrated;
}

void scanner::load_point_cloud() {}

void scanner::invokeIO(std::shared_ptr<command> comm) {  
    if(comm->code != COMM_IOSTART && !IOalive) return;
    if(comm->code == COMM_IOSTART && IOalive) return;

    if(comm->code == COMM_IOSTART || comm->code == COMM_IOSTOP) comm->execute(comm);
    
    commandq.enqueue(comm);
}

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
    int code=j["code"].get < int > ();

    switch(code) {
        case COMM_IOSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_iostart(sc, code)));
            break;
        case COMM_IOSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_iostop(sc, code)));        
            break;
        case COMM_VIDEOSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_videostart(sc, code)));
            break;
        case COMM_VIDEOSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_videostop(sc, code)));
            break;
        case COMM_CAMERACALIBSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_cameracalibstart(sc, code)));
            break;
        case COMM_CAMERACALIBSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_cameracalibstop(sc, code)));
            break;            
        case COMM_SCANNERCALIBSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_scannercalibstart(sc, code)));
            break;
        case COMM_SCANNERCALIBSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_scannercalibstop(sc, code)));
            break;        
        case COMM_SCANSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_scanstart(sc, code)));
            break; 
        case COMM_SCANSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_scanstop(sc, code)));
            break;
        case COMM_CONTROLLERSTART:
            sc.invokeIO(std::shared_ptr<command>(new command_microcontrollerstart(sc, code)));
            break; 
        case COMM_CONTROLLERSTOP:
            sc.invokeIO(std::shared_ptr<command>(new command_microcontrollerstop(sc, code)));
            break;
        case COMM_TOGGLELASER: {
            int state = j["state"].get<int>();
            sc.invokeIO(std::shared_ptr<command>(new command_laserset(sc, code, state)));
        }
            break;
        case COMM_ROTATE: {
            int direction = j["direction"].get<int>();
            sc.invokeIO(std::shared_ptr<command>(new command_rotate(sc, code, direction)));
        }
            break;
    }
}

void post_message(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    auto recipient = j["recipient"].get<std::string>();

    auto comm = [recipient,j]() {
        if(!recipient.compare("camera_thread")) {
            sc.camera.post_message_camera(j);
        }
    };

    sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, -1, comm)));
}


void keyboard_input(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    // int recipient = j["keycode"].get<int>();
    int keycode = j["keycode"].get<int>();

    auto comm = [keycode]() {
        sc.camera.set_key_camera(keycode);
    };

    sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, -1, comm)));
}

Napi::ThreadSafeFunction stremitTSFN;
void scanner::stremit(std::string e, std::string msg, bool blocking) {    
    auto callback = [e, msg]( Napi::Env env, Napi::Function jscb) {
        auto it = ev_handlers.find(e);
    
        if(it != ev_handlers.end()) it->second.Value().Call({Napi::String::New(env, msg)});
    };

    if(blocking) stremitTSFN.BlockingCall(callback);
    else stremitTSFN.NonBlockingCall(callback);
}

Napi::ThreadSafeFunction imemitTSFN;
void scanner::imemit(std::string e, uint8_t* imbase64, size_t len, bool blocking) {    
    auto callback = [e, imbase64, len]( Napi::Env env, Napi::Function jscb) {
        auto it = ev_handlers.find(e);

        if(it != ev_handlers.end()) it->second.Value().Call({Napi::Buffer<uint8_t>::New(env, imbase64,len, [](Napi::Env env, uint8_t* data) { delete data; })});
    };

    if(blocking) imemitTSFN.BlockingCall(callback);
    else imemitTSFN.NonBlockingCall(callback);
}

void scanner::imemit(std::string e, uint8_t* imbase64, std::string msg, size_t len, bool blocking) {    
    auto callback = [e,imbase64,msg,len]( Napi::Env env, Napi::Function jscb) {
        auto it = ev_handlers.find(e);

        if(it != ev_handlers.end()) it->second.Value().Call({Napi::String::New(env,msg),Napi::Buffer<uint8_t>::New(env, imbase64,len, [](Napi::Env env, uint8_t* data) { delete data; })});
    };

    if(blocking) imemitTSFN.BlockingCall(callback);
    else imemitTSFN.NonBlockingCall(callback);
}


void setprop(const Napi::CallbackInfo& info) {
    std::string jstr = info[0].As<Napi::String>().Utf8Value();
    nlohmann::json j = nlohmann::json::parse(jstr);
    int code = j["code"].get<int>();
    std::string prop = j["prop"].get<std::string>();

    if(!prop.compare(PROP_VIDEOALIVE)) {
            bool val = j["value"].get<bool>();

            auto comm = [val]() {
                boost::unique_lock<boost::mutex> lock(sc.camera.mtx_video_alive);
                sc.camera.video_alive = val;
                lock.unlock();
                nlohmann::json j;
                j["prop"] = PROP_VIDEOALIVE;
                j["value"] = val;
                sc.stremit(EV_PROPCHANGED, j.dump(), true);    
            };

            sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, code, comm)));
    }
    else if(!prop.compare(PROP_SCANNERCALIBRATED)) {
            bool val = j["value"].get<bool>();

            auto comm = [val]() {
                boost::unique_lock<boost::mutex> lock(sc.mtx_calibrated);
                sc.calibrated = val;
                lock.unlock();
                nlohmann::json j;
                j["prop"] = PROP_SCANNERCALIBRATED;
                j["value"] = val;
                sc.stremit(EV_PROPCHANGED, j.dump(), true);    
            };

            sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, code, comm)));
    } 
}

Napi::Value getprop(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    std::string prop = info[0].As<Napi::String>().Utf8Value();


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
            if(!prop.compare(PROP_VIDEOALIVE)) {
                boost::unique_lock<boost::mutex> lock(sc.camera.mtx_video_alive);
                ctx->deferred.Resolve(Napi::Boolean::New(ctx->deferred.Env(), sc.camera.video_alive));
            }            
            else if(!prop.compare(PROP_CAMERACALIBRATED)) {
                boost::unique_lock<boost::mutex> lock(sc.camera.mtx_calibrated);
                ctx->deferred.Resolve(Napi::Boolean::New(ctx->deferred.Env(), sc.camera.calibrated));
            }            
            else if(!prop.compare(PROP_SCANNERCALIBRATED)) {
                boost::unique_lock<boost::mutex> lock(sc.mtx_calibrated);
                ctx->deferred.Resolve(Napi::Boolean::New(ctx->deferred.Env(), sc.calibrated));
            }            
            else if(!prop.compare(PROP_CAMERA_CALIB_CAPTURES)) {
                // boost::unique_lock<boost::mutex> lock(sc.mtx_calibrated);
                ctx->deferred.Resolve(Napi::Number::New(ctx->deferred.Env(), sc.camera.calib.n_captures));
            }
            else if(!prop.compare(PROP_CAMERALIST)) {
                auto cam_list=camera::get_camera_list();
                nlohmann::json jarray = nlohmann::json::array();

                for(int i=0;i<cam_list.size();i++) {
                    nlohmann::json j;
                    j["id"]=cam_list[i].id;
                    j["name"]=cam_list[i].name;
                    jarray.push_back(j);
                }

                nlohmann::json j;
                j["data"] = jarray;
                ctx->deferred.Resolve(Napi::String::New(ctx->deferred.Env(), j.dump()));
            }
            else if(!prop.compare(PROP_SCANNER_CALIBRATION_DATA)) {
                auto plane_coeffs=sc.sccalib.laser_plane.coeffs();
                auto axis_origin=sc.sccalib.rotation_axis_origin;
                auto axis_direction=sc.sccalib.rotation_axis_direction;
                std::vector<double> laser_plane(plane_coeffs.data(), plane_coeffs.data() + plane_coeffs.rows() * plane_coeffs.cols()),
                rotation_axis_direction(axis_direction.data(), axis_direction.data() + axis_direction.rows() * axis_direction.cols()),
                rotation_axis_origin(axis_origin.data(), axis_origin.data() + axis_origin.rows() * axis_origin.cols());

                nlohmann::json j;
                j["laser_plane"] = laser_plane;
                j["rotation_axis_origin"] = rotation_axis_origin;
                j["rotation_axis_direction"] = rotation_axis_direction;                
                ctx->deferred.Resolve(Napi::String::New(ctx->deferred.Env(), j.dump()));
            }
            else if(!prop.compare(PROP_SCAN_RENDER_DATA)) {
                auto rotation_axis_radius=sc.sccalib.rotation_axis_radius;
                auto t=sc.sccalib.rotation_axis_origin;
                auto rotation_axis_direction=sc.sccalib.rotation_axis_direction;
                Eigen::Matrix3d R_axis = Eigen::AngleAxisd(0, rotation_axis_direction).toRotationMatrix();
                Eigen::Matrix<double,3,4> T_inv;
                T_inv.block<3, 3>(0, 0) = R_axis.transpose();
                T_inv.block<3, 1>(0, 3) = -R_axis.transpose() * t;
                Eigen::Vector3d proj=(-t).dot(rotation_axis_direction)*rotation_axis_direction+t;
                Eigen::Vector3d anchor_object=T_inv*Eigen::Vector4d(proj(0),proj(1),proj(2),1);
                Eigen::Vector3d camera_origin_object=T_inv*Eigen::Vector4d(0,0,0,1);
                std::vector<double> object_anchor(anchor_object.data(), anchor_object.data() + anchor_object.rows() * anchor_object.cols()),
                camera_origin(camera_origin_object.data(), camera_origin_object.data() + camera_origin_object.rows() * camera_origin_object.cols());

                nlohmann::json j;
                j["object_anchor"] = object_anchor;
                j["camera_origin"] = camera_origin;
                j["rotation_axis_radius"] = rotation_axis_radius;
                ctx->deferred.Resolve(Napi::String::New(ctx->deferred.Env(), j.dump()));
            }            
        };

        ctx->tsfn.NonBlockingCall(getprop);
        ctx->tsfn.Release();
    };

    sc.invokeIO(std::shared_ptr<command>(new command_lambda<decltype(comm)>(sc, COMM_GETPROP, comm)));

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
    exports.Set(Napi::String::New(env, "getProp"), 
                    Napi::Function::New(env, getprop));
    exports.Set(Napi::String::New(env, "postMessage"), 
                    Napi::Function::New(env, post_message));
    stremitTSFN = Napi::ThreadSafeFunction::New(env,  Napi::Function::New(env, [](const Napi::CallbackInfo& info){ return info.Env().Undefined(); } ), "stremit", 0, 1);
    imemitTSFN = Napi::ThreadSafeFunction::New(env,  Napi::Function::New(env, [](const Napi::CallbackInfo& info){ return info.Env().Undefined(); } ), "imemit", 2, 1);

    return exports;
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, init)
}
