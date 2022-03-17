#include <commands/command_mainstart.hpp>
#include <boost/thread.hpp>

namespace scanner {
    command_mainstart::command_mainstart(scanner* ctx, int code) : command(ctx, code) {};

    //TODO:rework
    void command_mainstart::execute() {
                ctx->set_flag_thread_main_alive(true);

                auto fn = [ctx=ctx]() {
                    ctx->commandq.clear();

                    while(ctx->get_flag_thread_main_alive()) {
                        try {
                            auto comm = ctx->commandq.dequeue();

                            std::cout<<"command code: "<<comm->code<<std::endl;

                            switch (comm->code)
                            {
                                case COMM_VIDEOCAPTURESTART:
                                    if(ctx->camera.get_flag_thread_video_capture_alive())  continue;
                                    break;
                                case COMM_VIDEOCAPTURESTOP:
                                    if(!ctx->camera.get_flag_thread_video_capture_alive())  continue;
                                    break;
                                case COMM_VIDEOOPENSTART:
                                    if(ctx->camera.get_flag_thread_video_open_alive())  continue;
                                    break;
                                case COMM_VIDEOOPENSTOP:
                                    if(!ctx->camera.get_flag_thread_video_open_alive())  continue;
                                    break;
                                case COMM_CAMERACALIBSTART:
                                    if(ctx->camera.get_flag_thread_camera_alive())  continue;
                                    break;
                                case COMM_CAMERACALIBSTOP:
                                    if(!ctx->camera.get_flag_thread_camera_alive())  continue;
                                    break;
                                case COMM_SCANNERCALIBSTART:
                                    if(ctx->get_flag_calibrating_scanner())  continue;
                                    break;
                                case COMM_SCANNERCALIBSTOP:
                                    if(!ctx->get_flag_calibrating_scanner())  continue;
                                    break;
                                case COMM_SCANSTART:
                                    if(ctx->get_flag_scanning())  continue;
                                    break;
                                case COMM_SCANSTOP:
                                    if(!ctx->get_flag_scanning())  continue;
                                    break;
                                default:
                                    break;
                            }

                            comm->execute();
                        }
                        catch(boost::thread_interrupted&) { }
                    }
                };

                ctx->stremit(EV_MAINSTART, "", true);
                ctx->thread_main = boost::thread{fn};
    }
}