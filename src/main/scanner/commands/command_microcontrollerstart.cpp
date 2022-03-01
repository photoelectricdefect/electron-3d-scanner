#include <commands/command_microcontrollerstart.hpp>

namespace scanner {
command_microcontrollerstart::command_microcontrollerstart(scanner* ctx, int code)
    : command(ctx, code) {}

void command_microcontrollerstart::execute()
{
    ctx->controller.set_flag_thread_controller_alive(true);

    auto fn = [ctx=ctx]() {
        ctx->controller.commandq.clear();
        bool running = true;

        while (running) {
            try {
                auto comm = ctx->controller.commandq.dequeue();

                if (comm->code == COMM_ROTATE) {
                    boost::unique_lock<boost::mutex> lock(ctx->mutex_scanning);

                    if (ctx->scanning)
                        continue;
                }

                comm->execute();
            }
            catch (boost::thread_interrupted&) {
                running = false;
            }
        }
    };

    ctx->stremit(EV_CONTROLLERSTART, "", true);
    ctx->controller.thread_controller = boost::thread{ fn };
}
}