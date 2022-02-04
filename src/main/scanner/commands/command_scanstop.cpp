#include <commands/command_scanstop.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    command_scanstop::command_scanstop(scanner& ctx, int code) : command(ctx, code) {}

    void command_scanstop::execute(std::shared_ptr<command> self) {
        ctx.camera.thread_camera.interrupt();
        ctx.camera.thread_camera.join();
        ctx.camera.thread_video.interrupt();
        ctx.camera.thread_video.join();
        ctx.scanning = false;
        boost::unique_lock<boost::mutex> lock(ctx.camera.mtx_video_alive);
        ctx.camera.video_alive = false;
        lock.unlock();
        ctx.camera.camera_alive = false;
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