#include <commands/command_iostop.hpp>

namespace scanner {
    command_iostop::command_iostop(scanner& ctx, int code) : command(ctx, code) {}
    command_iostop::command_iostop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

        void command_iostop::execute(std::shared_ptr<command> self) {
            self->ctx.thread_video.interrupt();
            self->ctx.thread_video.join();
            self->ctx.threadIO.interrupt();
            self->ctx.threadIO.join();
            self->ctx.set_IOalive(false);
            self->ctx.set_video_alive(false);
            self->ctx.stremit(EV_VIDEOSTOP, "", true);
            self->ctx.stremit(EV_IOSTOP, "", true);
        }
}