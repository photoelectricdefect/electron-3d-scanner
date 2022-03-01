#include <commands/command_videostart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    const int delay_open_capture_ms=5000;

    command_videostart::command_videostart(scanner* ctx, int code) : command(ctx, code) {}

    void command_videostart::execute() {
        ctx->camera.set_flag_thread_video_alive(true);
        // ctx->camera.set_flag_display_video(true);

        auto fn_video = [ctx=ctx]() {
            bool read_success=false;

            while(ctx->camera.get_flag_thread_video_alive()) {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(delay_open_capture_ms));
                boost::unique_lock<boost::mutex> lock_video_capture0(ctx->camera.mutex_video_capture);

                if (!read_success)
                {
                    int videoid=camera::get_videoid("USB 2.0 Camera: USB Camera");

                    if(videoid==-1) {
                        std::cerr<<"could not detect USB camera"<<std::endl;
                        continue;
                    }

                    ctx->camera.video_capture.open(videoid);
                    ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                    ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);         

                    if (!ctx->camera.video_capture.isOpened()) {
                        std::cerr << "error opening camera" << std::endl;
                        continue;
                    }
                }

                lock_video_capture0.unlock();

                try {
                    boost::unique_lock<boost::mutex> lock_display_video(ctx->camera.mutex_display_video);
                    
                    while(!ctx->camera.display_video)
                    {
                        ctx->camera.condition_display_video.wait(lock_display_video);
                    }

                    lock_display_video.unlock();
                    cv::Mat frame;

                    while (ctx->camera.get_flag_display_video())
                    {
                        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                        boost::unique_lock<boost::mutex> lock_video_capture1(ctx->camera.mutex_video_capture);
                        read_success=ctx->camera.video_capture.read(frame);

                        if (!read_success) {
                            std::cerr << "failed to read frame" << std::endl;
                            break;
                        }

                        lock_video_capture1.unlock();

                        if(ctx->get_flag_calibrating_scanner()||ctx->get_flag_scanning()) {
                            cv::Mat undistorted;

                            boost::unique_lock<boost::mutex> lock_KD(ctx->camera.camera_calibration.mutex_KD);
                            //TODO: lock K and D because can be changed while running if camera is calibrated
                            cv::undistort(frame, undistorted, ctx->camera.camera_calibration.K, ctx->camera.camera_calibration.D);
                            lock_KD.unlock();

                            uint8_t *data;
                            auto len = cv_helpers::mat2buffer(undistorted, data);
                            ctx->imemit(EV_IMUPDATE, data, len, true);
                        }
                        else {
                            uint8_t* data;
                            auto len = cv_helpers::mat2buffer(frame, data);
                            ctx->imemit(EV_IMUPDATE, data, len, true);
                        }
                    }
                }
                catch (boost::thread_interrupted &) {}
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