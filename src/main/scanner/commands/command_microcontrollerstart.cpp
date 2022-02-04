#include <commands/command_microcontrollerstart.hpp>

namespace scanner {
command_microcontrollerstart::command_microcontrollerstart(scanner& ctx, int code)
    : command(ctx, code)
{
}

void command_microcontrollerstart::execute(std::shared_ptr<command> self)
{
    ctx.controller.controller_alive = true;

    auto fn = [self]() {
        self->ctx.controller.commandq.clear();
        bool running = true;

        while (running) {
            try {
                auto comm = self->ctx.controller.commandq.dequeue();

                if (comm->code == COMM_ROTATE) {
                    boost::unique_lock<boost::mutex> lock(self->ctx.mtx_scanning);

                    if (self->ctx.scanning)
                        continue;
                }

                comm->execute(comm);
            }
            catch (boost::thread_interrupted&) {
                running = false;
            }
        }
    };

    ctx.stremit(EV_CONTROLLERSTART, "", true);
    ctx.controller.thread_controller = boost::thread{ fn };
}
}