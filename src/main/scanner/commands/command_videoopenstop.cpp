#include <commands/command_videoopenstop.hpp>

namespace scanner {
    command_videoopenstop::command_videoopenstop(scanner* ctx, int code) : command(ctx, code) {}

    void command_videoopenstop::execute() {        
        ctx->camera.set_flag_thread_video_open_alive(false);
        ctx->camera.notify_video_closed();
        ctx->camera.thread_video_open.interrupt();
        ctx->camera.thread_video_open.join();
    }
}