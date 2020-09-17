#include <commands/command_iostart.hpp>
#include <boost/thread.hpp>

namespace scanner {
    command_iostart::command_iostart(scanner& ctx, int code) : command(ctx, code) {};
    command_iostart::command_iostart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_iostart::execute(std::shared_ptr<command> self) {
                self->ctx.set_IOalive(true);

                auto fn = [self]() {
                    self->ctx.commandq.clear();
                    bool running = true;

                    while(running) {
                        try {
                            boost::this_thread::interruption_point();
                            std::shared_ptr<command> comm = self->ctx.commandq.dequeue();
                            comm->execute(comm);
                        }
                        catch(boost::thread_interrupted&) { running = false; }
                    }
                };

                self->ctx.threadIO = boost::thread{fn};
                self->ctx.stremit(EV_IOSTART, "", true);
    }
}