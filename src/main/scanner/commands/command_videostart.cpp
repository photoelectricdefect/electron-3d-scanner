#include <commands/command_videostart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    const int delay_open_capture_ms=5000;

    bool wait_on_video_conditions(scanner* ctx) {
        boost::unique_lock<boost::mutex> lock_video_conditions(ctx->camera.mutex_video_conditions);

        while(!ctx->camera.display_video||ctx->camera.calibrating_camera) {
            ctx->camera.condition_video_conditions.wait(lock_video_conditions);
        }

        return true;
    }

    command_videostart::command_videostart(scanner* ctx, int code) : command(ctx, code) {}

    void command_videostart::execute() {
        ctx->camera.set_flag_thread_video_alive(true);
        // ctx->camera.set_flag_display_video(true);

        auto fn_video = [ctx=ctx]() {
            bool video_device_open=false;

            const auto state_open_video_device = [ctx=ctx,delay_open_capture_ms](bool& video_device_open) {
                try {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(delay_open_capture_ms));
                    boost::unique_lock<boost::mutex> lock_video_capture(ctx->camera.mutex_video_capture);

                    if (!video_device_open)
                    {                    
                        camera::camera_info selected_camera_info;
                        ctx->camera.get_selected_camera_info(selected_camera_info);

                        std::cout<<"vidoeidididi"<<selected_camera_info.id<<std::endl;
                        std::cout<<"nnnname"<<selected_camera_info.name<<std::endl;

                        ctx->camera.video_capture.open(selected_camera_info.id);
                        ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                        ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);         
                        video_device_open=ctx->camera.video_capture.isOpened();
                    }

                    lock_video_capture.unlock();
                
                    if (!video_device_open) {
                        std::cerr << "error opening camera" << std::endl;
                    }
                }
                catch(boost::thread_interrupted &ex) {
                    video_device_open=false;
                }
            };

            const auto state_grab_frames = [ctx=ctx](bool& video_device_open) {
                try {
                    while (wait_on_video_conditions(ctx))
                    {
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                        boost::unique_lock<boost::mutex> lock_video_conditions(ctx->camera.mutex_video_conditions);

                        std::cout<<"grabbing frames"<<std::endl;

                        if(!ctx->camera.display_video||ctx->camera.calibrating_camera) {
                            continue;
                        }

                        cv::Mat frame;
                        boost::unique_lock<boost::mutex> lock_video_capture(ctx->camera.mutex_video_capture);
                        video_device_open=ctx->camera.video_capture.read(frame);
                        lock_video_capture.unlock();

                        if (!video_device_open) {
                            break;
                        }

                        cv::Mat frame_out;
                        
                        {
                            boost::unique_lock<boost::mutex> lock_camera_calibrated(ctx->camera.mutex_camera_calibrated);

                            if(ctx->camera.camera_calibrated) {
                                cv::undistort(frame, frame_out, ctx->camera.camera_calibration.K, ctx->camera.camera_calibration.D);
                            }
                            else {
                                frame_out=frame;
                            }
                        }
                                                                        
                        uint8_t* data;
                        auto len = cv_helpers::mat2buffer(frame_out, data);
                        ctx->imemit(EV_IMUPDATE, data, len, true);
                    }
                }
                catch (boost::thread_interrupted &ex) {
                    video_device_open=false;
                }
            };

            while(ctx->camera.get_flag_thread_video_alive()) {
                state_open_video_device(video_device_open);
                
                if(video_device_open) {
                    state_grab_frames(video_device_open);
                }
            }
        };

        // nlohmann::json j;
        // j["prop"] = PROP_DISPLAYVIDEO;
        // j["value"] = true;
        // ctx->stremit(EV_PROPCHANGED, j.dump(), true);
        ctx->stremit(EV_VIDEOSTART, "", true);
        ctx->camera.thread_video = boost::thread{fn_video};
    }
}