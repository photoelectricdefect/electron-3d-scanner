#include <commands/command_videoopenstart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    const int delay_open_video_ms=5000;

    camera::camera_info wait_on_conditions(scanner* ctx) {
        boost::unique_lock<boost::mutex> lock(ctx->camera.mutex_video_open);

        while(ctx->camera.video_open||ctx->camera.selected_camera_info==nullptr) {
            ctx->camera.condition_video_open.wait(lock);
        }

        return *ctx->camera.selected_camera_info;
    }

    command_videoopenstart::command_videoopenstart(scanner* ctx, int code) : command(ctx, code) {}

    void command_videoopenstart::execute() {
        ctx->camera.set_flag_thread_video_open_alive(true);

        auto fn_video_open = [ctx=ctx]() {
            while(ctx->camera.get_flag_thread_video_open_alive()) {
                try {
                    auto selected_camera_info=wait_on_conditions(ctx);
                    boost::unique_lock<boost::mutex> lock_video_capture(ctx->camera.mutex_video_capture);

                    std::cout<<"vidoeidididi"<<selected_camera_info.id<<std::endl;
                    std::cout<<"nnnname"<<selected_camera_info.name<<std::endl;

                    ctx->camera.video_capture.open(selected_camera_info.id);
                    ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                    ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                    boost::unique_lock<boost::mutex> lock_video_open(ctx->camera.mutex_video_open);         
                    ctx->camera.video_open=ctx->camera.video_capture.isOpened();

                    // if (!ctx->camera.video_open)
                    // {                    
                    //     camera::camera_info selected_camera_info;
                    //     ctx->camera.get_selected_camera_info(selected_camera_info);

                    //     // std::cout<<"vidoeidididi"<<selected_camera_info.id<<std::endl;
                    //     // std::cout<<"nnnname"<<selected_camera_info.name<<std::endl;

                    //     ctx->camera.video_capture.open(selected_camera_info.id);
                    //     ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                    //     ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);         
                    //     ctx->camera.set_flag_video_open(ctx->camera.video_capture.isOpened());
                    // }

                    //delete                
                    if (!ctx->camera.video_open) {
                        std::cerr << "error opening camera" << std::endl;
                    }
                }
                catch(boost::thread_interrupted &ex) {}
            }
        };

        ctx->camera.thread_video_open = boost::thread{fn_video_open};
    }
}