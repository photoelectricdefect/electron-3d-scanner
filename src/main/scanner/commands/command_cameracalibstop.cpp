#include <commands/command_cameracalibstop.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {
    command_cameracalibstop::command_cameracalibstop(scanner& ctx, int code) : command(ctx, code) {};
    command_cameracalibstop::command_cameracalibstop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_cameracalibstop::execute(std::shared_ptr<command> self) {
		ctx.thread_camera.interrupt();
		ctx.thread_camera.join();
		ctx.camera_alive = false;
    ctx.video_alive = false;
    ctx.calibratingcamera = false;
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