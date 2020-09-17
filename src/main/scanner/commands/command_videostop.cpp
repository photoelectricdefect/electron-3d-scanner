#include <commands/command_videostop.hpp>

namespace scanner {
    command_videostop::command_videostop(scanner& ctx, int code) : command(ctx, code) {}
    command_videostop::command_videostop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

    void command_videostop::execute(std::shared_ptr<command> self) {
        self->ctx.thread_video.interrupt();
        self->ctx.thread_video.join();
        self->ctx.set_video_alive(false);
        self->ctx.stremit(EV_VIDEOSTOP, "", true);
    }
}