#include <commands/command_iostart.hpp>
#include <commands/command_videostop.hpp>
#include <boost/thread.hpp>

namespace scanner {
    command_iostart::command_iostart(scanner& ctx, int code) : command(ctx, code) {};
    command_iostart::command_iostart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_iostart::execute(std::shared_ptr<command> self) {
                ctx.IOalive = true;

                auto fn = [self]() {
                    self->ctx.commandq.clear();
                    bool running = true;

                    while(running) {
                        try {
                            auto comm = self->ctx.commandq.dequeue();

                            // std::cout << "code: " << comm->code << std::endl; 
                            // std::cout << "cameracalib: " << self->ctx.calibratingcamera << std::endl; 
                            // std::cout << "videoalive: " << self->ctx.video_alive << std::endl; 

                            if(comm->code == COMM_CAMERACALIBSTART) {
                                if(self->ctx.camera.calibrating) continue;
                                else if(self->ctx.scanning); //TODO
                                else if(self->ctx.camera.video_alive) {
                                    std::shared_ptr<command> stop(new command_videostop(self->ctx, COMM_VIDEOSTOP));
                                    stop->execute(stop);
                                }
                            }
                            else if(comm->code == COMM_CAMERACALIBSTOP) {
                                if(!self->ctx.camera.calibrating) continue;
                            } 
                            else if(comm->code == COMM_VIDEOSTART) {
                                if(self->ctx.camera.calibrating || self->ctx.scanning || self->ctx.camera.video_alive) continue;
                            } 
                            else if(comm->code == COMM_VIDEOSTOP) {
                                if(self->ctx.camera.calibrating || self->ctx.scanning || !self->ctx.camera.video_alive) continue;
                            } 

                            comm->execute(comm);
                        }
                        catch(boost::thread_interrupted&) { running = false; }
                    }
                };

                ctx.stremit(EV_IOSTART, "", true);
                ctx.threadIO = boost::thread{fn};
    }
}