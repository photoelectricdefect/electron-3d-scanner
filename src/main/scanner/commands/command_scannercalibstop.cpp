#include <commands/command_scannercalibstop.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {
    command_scannercalibstop::command_scannercalibstop(scanner& ctx, int code) : command(ctx, code) {};

    void command_scannercalibstop::execute(std::shared_ptr<command> self) {
        ctx.camera.thread_camera.interrupt();
        ctx.camera.thread_camera.join();
        ctx.camera.thread_video.interrupt();
        ctx.camera.thread_video.join();
        std::cout<<"before"<<std::endl;
        boost::unique_lock<boost::mutex> lock(ctx.camera.mtx_video_alive);
        ctx.camera.video_alive = false;
        lock.unlock();
        std::cout<<"after"<<std::endl;
        ctx.camera.camera_alive = false;
        ctx.calibrating = false;
        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        j["prop"] = PROP_CALIBRATINGSCANNER;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);  
        ctx.stremit(EV_SCANNERCALIBSTOP, "", true);
    }
}