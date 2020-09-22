#include <commands/command_iostop.hpp>

namespace scanner {
    command_iostop::command_iostop(scanner& ctx, int code) : command(ctx, code) {}
    command_iostop::command_iostop(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

        void command_iostop::execute(std::shared_ptr<command> self) {
            ctx.threadIO.interrupt();
            ctx.threadIO.join();
            ctx.thread_camera.interrupt();
            ctx.thread_camera.join();
            ctx.IOalive = false;
            ctx.video_alive = false;
            ctx.camera_alive = false;
            ctx.stremit(EV_VIDEOSTOP, "", true);
            ctx.stremit(EV_IOSTOP, "", true);
        }
}