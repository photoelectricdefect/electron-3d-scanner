#include <commands/command_cameracalibstop.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {
    command_cameracalibstop::command_cameracalibstop(scanner* ctx, int code) : command(ctx, code) {};

    void command_cameracalibstop::execute() {
      ctx->camera.set_flag_calibrating_camera(false);
      ctx->camera.set_flag_thread_camera_alive(false);
		  ctx->camera.thread_camera.interrupt();
		  ctx->camera.thread_camera.join();
      nlohmann::json j;
      j["prop"] = PROP_CALIBRATINGCAMERA;
      j["value"] = false;
      ctx->stremit(EV_PROPCHANGED, j.dump(), true); 
		  ctx->stremit(EV_CAMERACALIBSTOP, "", true);
    }
}