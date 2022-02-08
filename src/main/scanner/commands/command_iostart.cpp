#include <commands/command_iostart.hpp>
#include <commands/command_videostop.hpp>
#include <boost/thread.hpp>

namespace scanner {
    command_iostart::command_iostart(scanner& ctx, int code) : command(ctx, code) {};

    void command_iostart::execute(std::shared_ptr<command> self) {
                ctx.IOalive = true;

                auto fn = [self]() {
                    self->ctx.commandq.clear();
                    bool running = true;

                    while(running) {
                        try {
                            auto comm = self->ctx.commandq.dequeue();

                            std::cout<<"command code: "<<comm->code<<std::endl;

                            if(comm->code == COMM_CAMERACALIBSTART) {
                                if(self->ctx.camera.calibrating||self->ctx.calibrating||self->ctx.scanning) continue;
                                else if(self->ctx.camera.video_alive) {
                                    std::shared_ptr<command> stop(new command_videostop(self->ctx, COMM_VIDEOSTOP));
                                    stop->execute(stop);
                                }
                            }
                            else if(comm->code == COMM_CAMERACALIBSTOP) {
                                if(!self->ctx.camera.calibrating) continue;
                            } 
                            else if(comm->code == COMM_SCANNERCALIBSTART) {
                                if(self->ctx.camera.calibrating||self->ctx.calibrating||self->ctx.scanning) continue;
                                else if(self->ctx.camera.video_alive) {
                                    std::shared_ptr<command> stop(new command_videostop(self->ctx, COMM_VIDEOSTOP));
                                    stop->execute(stop);
                                }
                            }
                            else if(comm->code == COMM_SCANSTART) {
                                if(self->ctx.camera.calibrating||self->ctx.calibrating||self->ctx.scanning) continue;
                                else if(self->ctx.camera.video_alive) {
                                    std::shared_ptr<command> stop(new command_videostop(self->ctx, COMM_VIDEOSTOP));
                                    stop->execute(stop);
                                }
                            } 
                            else if(comm->code == COMM_SCANSTOP) {
                                if(!self->ctx.scanning) continue;
                            }  
                            else if(comm->code == COMM_SCANNERCALIBSTOP) {
                                if(!self->ctx.calibrating) continue;
                            } 
                            else if(comm->code == COMM_VIDEOSTART) {
                                if(self->ctx.camera.calibrating||self->ctx.scanning||self->ctx.calibrating||self->ctx.camera.video_alive) continue;
                            } 
                            else if(comm->code == COMM_VIDEOSTOP) {
                                if(self->ctx.camera.calibrating||self->ctx.scanning||self->ctx.calibrating||!self->ctx.camera.video_alive) continue;
                            } 
                            else if(comm->code == COMM_ROTATE) {
                                if(self->ctx.camera.calibrating||self->ctx.scanning||self->ctx.calibrating) continue;
                            } 
                            else if(comm->code == COMM_TOGGLELASER) {
                                if(self->ctx.camera.calibrating||self->ctx.scanning||self->ctx.calibrating) continue;
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