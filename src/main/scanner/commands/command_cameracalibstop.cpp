#include <commands/command_cameracalibstop.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {
    command_cameracalibstop::command_cameracalibstop(scanner& ctx, int code) : command(ctx, code) {};

    void command_cameracalibstop::execute(std::shared_ptr<command> self) {
		ctx.camera.thread_camera.interrupt();
		ctx.camera.thread_camera.join();
		ctx.camera.camera_alive = false;
    boost::unique_lock<boost::mutex> lock(ctx.camera.mtx_video_alive);
    ctx.camera.video_alive = false;
    lock.unlock();
    ctx.camera.calibrating = false;
    nlohmann::json j;
    j["prop"] = PROP_CALIBRATINGCAMERA;
    j["value"] = false;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true); 
    j["prop"] = PROP_VIDEOALIVE;
    j["value"] = false;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);   
		ctx.stremit(EV_CAMERACALIBSTOP, "", true);
    }
}