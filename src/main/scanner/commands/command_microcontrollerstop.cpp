#include <commands/command_microcontrollerstop.hpp>

namespace scanner {
    command_microcontrollerstop::command_microcontrollerstop(scanner& ctx, int code) : command(ctx, code) {}

    void command_microcontrollerstop::execute(std::shared_ptr<command> self) {        
        ctx.controller.thread_controller.interrupt();
        ctx.controller.thread_controller.join();
        ctx.controller.controller_alive = false;
        ctx.stremit(EV_CONTROLLERSTOP, "", true);
    }
}