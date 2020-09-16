#include <commands/command_iostop.hpp>

namespace scanner {
    command_iostop::command_iostop(scanner ctx, int code) : command(ctx, code) {}
    command_iostop::command_iostop(scanner ctx, jcommand jcomm) : command(ctx, jcomm) {}

        void command_iostop::execute() {
            _ctx.thread_video.interrupt();
            _ctx.thread_video.join();
            _ctx.threadIO.interrupt();
            _ctx.threadIO.join();
            _ctx.IOalive = false;
            _ctx.video_alive = false;
            _ctx.stremit(EV_VIDEOSTOP, "", true);
            _ctx.stremit(EV_IOSTOP, "", true);
        }
}