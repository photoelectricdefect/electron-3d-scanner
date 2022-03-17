#include <commands/command_mainstop.hpp>

namespace scanner {
    command_mainstop::command_mainstop(scanner* ctx, int code) : command(ctx, code) {}
        void command_mainstop::execute() {
            ctx->set_flag_thread_main_alive(false);
            ctx->thread_main.interrupt();
            ctx->thread_main.join();
            
            ctx->camera.set_flag_thread_camera_alive(false);
            ctx->camera.thread_camera.interrupt();
            ctx->camera.thread_camera.join();

            //TODO:fix this
            ctx->controller.thread_controller_alive=false;
            ctx->controller.thread_controller.interrupt();
            ctx->controller.thread_controller.join();            

            ctx->camera.set_flag_thread_video_open_alive(false);        
            ctx->camera.thread_video_open.interrupt();
            ctx->camera.thread_video_open.join();            

            ctx->camera.set_flag_thread_video_capture_alive(false);        
            ctx->camera.thread_video_capture.interrupt();
            ctx->camera.thread_video_capture.join();            

            ctx->stremit(EV_VIDEOCAPTURESTOP, "", true);
            ctx->stremit(EV_CONTROLLERSTOP, "", true);
            ctx->stremit(EV_MAINSTOP, "", true);
        }
}