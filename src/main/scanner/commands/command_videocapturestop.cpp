#include <commands/command_videocapturestop.hpp>

namespace scanner {
    command_videocapturestop::command_videocapturestop(scanner* ctx, int code) : command(ctx, code) {}

    void command_videocapturestop::execute() {        
        ctx->camera.set_flag_thread_video_capture_alive(false);
        ctx->camera.thread_video_capture.interrupt();
        ctx->camera.thread_video_capture.join();
        ctx->stremit(EV_VIDEOCAPTURESTOP, "", true);
    }
}