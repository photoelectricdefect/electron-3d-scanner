#include <commands/command_scannercalibstop.hpp>
#include <commands/command_videocapturestop.hpp>
#include <boost/thread.hpp>
#include <json.hpp>

namespace scanner {
    command_scannercalibstop::command_scannercalibstop(scanner* ctx, int code) : command(ctx, code) {};

    void command_scannercalibstop::execute() {
        command_videocapturestop comm_video_capture_stop(ctx,COMM_VIDEOCAPTURESTOP);
        comm_video_capture_stop.execute();

        ctx->set_flag_calibrating_scanner(false);
        ctx->camera.set_flag_thread_camera_alive(false);
        ctx->camera.thread_camera.interrupt();
        ctx->camera.thread_camera.join();
        nlohmann::json j;
        j["prop"] = PROP_CALIBRATINGSCANNER;
        j["value"] = false;
        ctx->stremit(EV_PROPCHANGED, j.dump(), true);  
        ctx->stremit(EV_SCANNERCALIBSTOP, "", true);
    }
}