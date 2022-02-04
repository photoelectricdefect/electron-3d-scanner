#include <commands/command_videostop.hpp>

namespace scanner {
    command_videostop::command_videostop(scanner& ctx, int code) : command(ctx, code) {}

    void command_videostop::execute(std::shared_ptr<command> self) {        
        ctx.camera.thread_video.interrupt();
        ctx.camera.thread_video.join();
        boost::unique_lock<boost::mutex> lock(ctx.camera.mtx_video_alive);
        ctx.camera.video_alive = false;
        lock.unlock();
        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = false;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true); 
        ctx.stremit(EV_VIDEOSTOP, "", true);
    }
}