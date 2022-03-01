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

            ctx->camera.set_flag_thread_video_alive(false);        
            ctx->camera.set_flag_display_video(false);        
            ctx->camera.thread_video.interrupt();
            ctx->camera.thread_video.join();            

            ctx->stremit(EV_VIDEOSTOP, "", true);
            ctx->stremit(EV_CONTROLLERSTOP, "", true);
            ctx->stremit(EV_MAINSTOP, "", true);
        }
}