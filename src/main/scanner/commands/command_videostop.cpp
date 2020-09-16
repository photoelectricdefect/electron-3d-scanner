#include <commands/command_videostop.hpp>

namespace scanner {
    command_videostop::command_videostop(scanner ctx, int code) : command(ctx, code) {}
    command_videostop::command_videostop(scanner ctx, jcommand jcomm) : command(ctx, jcomm) {}

    void command_videostop::execute() {
        _ctx.thread_video.interrupt();
        _ctx.thread_video.join();
        _ctx.video_alive = false;
        _ctx.stremit(EV_VIDEOSTOP, "", true);
    }
}