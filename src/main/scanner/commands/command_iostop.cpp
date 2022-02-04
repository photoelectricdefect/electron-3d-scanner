#include <commands/command_iostop.hpp>

namespace scanner {
    command_iostop::command_iostop(scanner& ctx, int code) : command(ctx, code) {}

        //TODO: figure out how to stop all running processes like scanning....
        void command_iostop::execute(std::shared_ptr<command> self) {
            ctx.threadIO.interrupt();
            ctx.threadIO.join();
            ctx.camera.thread_camera.interrupt();
            ctx.camera.thread_camera.join();
            ctx.controller.thread_controller.interrupt();
            ctx.controller.thread_controller.join();
            ctx.IOalive = false;
            ctx.camera.video_alive = false;
            ctx.camera.camera_alive = false;
            ctx.controller.controller_alive=false;
            ctx.stremit(EV_VIDEOSTOP, "", true);
            ctx.stremit(EV_CONTROLLERSTOP, "", true);
            ctx.stremit(EV_IOSTOP, "", true);
        }
}