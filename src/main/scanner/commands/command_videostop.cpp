#include <commands/command_videostop.hpp>

namespace scanner {
    command_videostop::command_videostop(scanner* ctx, int code) : command(ctx, code) {}

    void command_videostop::execute() {        
        ctx->camera.set_flag_display_video(false);
        ctx->camera.set_flag_thread_video_alive(false);
        ctx->camera.thread_video.interrupt();
        ctx->camera.thread_video.join();
        nlohmann::json j;
        j["prop"] = PROP_DISPLAYVIDEO;
        j["value"] = false;
        ctx->stremit(EV_PROPCHANGED, j.dump(), true);
        ctx->stremit(EV_VIDEOSTOP, "", true);
    }
}