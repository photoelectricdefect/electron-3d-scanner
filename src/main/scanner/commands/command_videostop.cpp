#include <commands/command_videostop.hpp>

namespace scanner {
    command_videostop::command_videostop(scanner& ctx, int code) : command(ctx, code) {}
    command_videostop::command_videostop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

    void command_videostop::execute(std::shared_ptr<command> self) {        
        ctx.camera.thread_camera.interrupt();
        ctx.camera.thread_camera.join();
        ctx.camera.video_alive = false;
        ctx.camera.thread_alive = false;
        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true); 
        ctx.stremit(EV_VIDEOSTOP, "", true);
    }
}