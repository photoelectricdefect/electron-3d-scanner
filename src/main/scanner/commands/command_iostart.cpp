#include <commands/command_iostart.hpp>
#include <boost/thread.hpp>

namespace scanner {
    command_iostart::command_iostart(scanner ctx, int code) : command(ctx, code) {};
    command_iostart::command_iostart(scanner ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_iostart::execute() {
                _ctx.IOalive = true;

                auto fn = [&_ctx]() {
                    _ctx.commandq.clear();
                    bool running = true;

                    while(running) {
                        try {
                            boost::this_thread::interruption_point(); 
                            command comm = _ctx.commandq.dequeue();
                            comm.execute();
                        }
                        catch(boost::thread_interrupted&) { running = false; }
                    }
                };

                _ctx.threadIO = boost::thread{fn};
                _ctx.stremit(EV_IOSTART, "", true);
    }
}