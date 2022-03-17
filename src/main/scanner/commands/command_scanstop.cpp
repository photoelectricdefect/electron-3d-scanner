#include <commands/command_scanstop.hpp>
#include <commands/command_videocapturestop.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    command_scanstop::command_scanstop(scanner* ctx, int code) : command(ctx, code) {}

    void command_scanstop::execute() {
        command_videocapturestop comm_video_capture_stop(ctx,COMM_VIDEOCAPTURESTOP);
        comm_video_capture_stop.execute();
        
        ctx->set_flag_scanning(false);
        ctx->camera.set_flag_thread_camera_alive(false);
        ctx->camera.thread_camera.interrupt();
        ctx->camera.thread_camera.join();
        nlohmann::json j;
        j["prop"] = PROP_SCANNING;
        j["value"] = false;
        ctx->stremit(EV_PROPCHANGED, j.dump(), true);  
        ctx->stremit(EV_SCANSTOP, "", true);
    }
}