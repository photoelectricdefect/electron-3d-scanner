#include <commands/command_scanstop.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    command_scanstop::command_scanstop(scanner& ctx, int code) : command(ctx, code) {}
    command_scanstop::command_scanstop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

    void command_scanstop::execute(std::shared_ptr<command> self) {
        ctx.camera.thread_camera.interrupt();
        ctx.camera.thread_camera.join();
        ctx.scanning = false;
        ctx.camera.video_alive = false;
        ctx.camera.thread_alive = false;
        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        j["prop"] = PROP_SCANNING;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);  
        ctx.stremit(EV_SCANSTOP, "", true);
    }
}