#include <commands/command_videocapturestart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    const int delay_open_video_ms=5000;

    bool wait_on_video_open(scanner* ctx) {
        boost::unique_lock<boost::mutex> lock(ctx->camera.mutex_video_open);

        while(!ctx->camera.video_open) {
            ctx->camera.condition_video_open.wait(lock);
        }

        return true;
    }

    command_videocapturestart::command_videocapturestart(scanner* ctx, int code) : command(ctx, code) {}

    void command_videocapturestart::execute() {
        ctx->camera.set_flag_thread_video_capture_alive(true);

        auto fn_video_capture = [ctx=ctx]() {
            while(ctx->camera.get_flag_thread_video_capture_alive()) {
                try {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                    wait_on_video_open(ctx);

                    cv::Mat frame;
                    boost::unique_lock<boost::mutex> lock_video_capture(ctx->camera.mutex_video_capture);
                    bool video_device_open=ctx->camera.video_capture.read(frame)
                    ctx->camera.set_flag_video_open(video_device_open);
                        
                    if(!video_device_open) {
                        continue;
                    }

                    lock_video_capture.unlock();
                    
                    cv::Mat frame_out;
                    boost::unique_lock<boost::mutex> lock_camera_calibrated(ctx->camera.mutex_camera_calibrated);

                    if(ctx->camera.camera_calibrated) {
                        cv::undistort(frame, frame_out, ctx->camera.camera_calibration.K, ctx->camera.camera_calibration.D);
                    }
                    else {
                        frame_out=frame;
                    }

                    lock_camera_calibrated.unlock();
                                                      
                    uint8_t* data;
                    auto len = cv_helpers::mat2buffer(frame_out, data);
                    ctx->imemit(EV_IMUPDATE, data, len, true);
                }
                catch (boost::thread_interrupted &ex) {}
            }
        };

        // nlohmann::json j;
        // j["prop"] = PROP_DISPLAYVIDEO;
        // j["value"] = true;
        // ctx->stremit(EV_PROPCHANGED, j.dump(), true);
        ctx->stremit(EV_VIDEOCAPTURESTART, "", true);
        ctx->camera.thread_video_capture = boost::thread{fn_video_capture};
    }
}