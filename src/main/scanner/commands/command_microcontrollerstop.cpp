#include <commands/command_microcontrollerstop.hpp>

namespace scanner {
    command_microcontrollerstop::command_microcontrollerstop(scanner* ctx, int code) : command(ctx, code) {}

    void command_microcontrollerstop::execute() {        
        ctx->controller.thread_controller.interrupt();
        ctx->controller.thread_controller.join();
        ctx->controller.set_flag_thread_controller_alive(false);
        ctx->stremit(EV_CONTROLLERSTOP, "", true);
    }
}