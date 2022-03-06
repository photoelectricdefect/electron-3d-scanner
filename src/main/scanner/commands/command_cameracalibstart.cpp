#include <commands/command_cameracalibstart.hpp>
#include <commands/command_cameracalibstop.hpp>
#include <commands/command_videostart.hpp>
#include <commands/command_lambda.hpp>
#include <helpers/cv_helpers.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/thread.hpp>
#include <json.hpp>
#include <vector>

namespace scanner {
const int delay_open_capture_ms=5000;

command_cameracalibstart::command_cameracalibstart(scanner* ctx, int code) : command(ctx, code){};

void command_cameracalibstart::execute()
{
    auto fn_camera = [ctx=ctx]()
    {
        const bool release_object=true;
        int n_captures=ctx->camera.camera_calibration.n_captures;
        auto pattern_size=ctx->camera.camera_calibration.pattern_size;
        int patternh = pattern_size.height,
            patternw = pattern_size.width;
        double squareh = ctx->camera.camera_calibration.square_size,
            squarew = ctx->camera.camera_calibration.square_size;

        std::vector<std::vector<cv::Point2f>> image_board_points;
        std::vector<std::vector<cv::Point3f>> world_board_points;

        for(int c=0;c<n_captures;c++) {
            std::vector<cv::Point3f> board_points;
            
            for (int i = 0; i < patternh; i++) {
                for (int j = 0; j < patternw; j++) {
                    board_points.push_back(cv::Point3f(j*squarew, i*squareh, 0));
                }
            }
        
            world_board_points.push_back(board_points);
        }

        // auto imupdate = [](const scanner* ctx,const cv::Mat frame) {
        //     boost::unique_lock<boost::mutex> lock(ctx->camera.mtx_video_alive);

        //     if (ctx->camera.video_alive) {
        //         uint8_t* data;
        //         auto len = cv_helpers::mat2buffer(frame, data);
        //         ctx->imemit(EV_IMUPDATE, data, len, true);
        //     }
        // };

        cv::Mat frame, gray;
        bool read_success=false;
        int current_captures = 0;

        while (ctx->camera.get_flag_thread_camera_alive()) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(delay_open_capture_ms));

            if(!read_success) {
                camera::camera_info selected_camera_info;
                ctx->camera.get_selected_camera_info(selected_camera_info);
                ctx->camera.video_capture.open(selected_camera_info.id);
                ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1600);          
                ctx->camera.video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);         

                if (!ctx->camera.video_capture.isOpened()) {
                    std::cerr << "error opening camera" << std::endl;
                    continue;
                }
            }

            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                int keycode=-1;                
                nlohmann::json message;
                bool message_recieved=ctx->camera.try_get_message_thread_camera(message);

                if(message_recieved) {
                    std::string type=message["type"];

                    if(!type.compare("keyup")) {
                        keycode=message["keycode"].get<int>();                                
                    }
                }
                
                read_success=ctx->camera.video_capture.read(frame);

                if (!read_success) {
                    continue;
                }

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Point2f> image_points;
                bool found = cv::findChessboardCorners(gray, pattern_size, image_points,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

                if (found) {
                    cv::cornerSubPix(gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

                    if (keycode == KEYCODE_SPACE) {
                        image_board_points.push_back(image_points);
                        current_captures++;
                        ctx->stremit(EV_CAMERACALIBCAPTURED, "", true);

                        if (current_captures >= n_captures)
                            break;
                    }

                    cv::drawChessboardCorners(frame, pattern_size, cv::Mat(image_points), found);
                }

                uint8_t* data;
                auto len = cv_helpers::mat2buffer(frame, data);
                ctx->imemit(EV_IMUPDATE, data, len, true);
            }
            catch (boost::thread_interrupted&) {}
        }

        if (current_captures >= n_captures) {
            cv::Mat rvecs, tvec,K,D;
            std::vector<cv::Point3f> new_world_board_points;
            int i_fixed_point = -1;

            if (release_object){
                i_fixed_point = patternw - 1;
            }

            std::cout<<"rmsRO:"<<cv::calibrateCameraRO(world_board_points, image_board_points, frame.size(), i_fixed_point,K, D, rvecs, tvec,new_world_board_points)<<std::endl;
            
            boost::unique_lock<boost::mutex> lock_camera_calibrated(ctx->camera.mutex_camera_calibrated);
            ctx->camera.camera_calibration.K=K;
            ctx->camera.camera_calibration.D=D;
            ctx->camera.camera_calibrated=true;
            lock_camera_calibrated.unlock();

            // std::cout<<"KCAM:"<<K<<std::endl;
            // std::cout<<"DCAM:"<<D<<std::endl;
            ctx->camera.camera_calibration.save("cameracalib.json");
            // ctx->camera.set_flag_camera_calibrated(true);
            nlohmann::json j;
            j["prop"] = "cameracalibrated";
            j["value"] = true;
            ctx->stremit(EV_PROPCHANGED, j.dump(), true);        
        }
    
        ctx->thread_main_invoke(std::shared_ptr<command>(new command_cameracalibstop(ctx, COMM_CAMERACALIBSTOP)));
    };

    ctx->camera.set_flag_display_video(true);
    ctx->camera.set_flag_calibrating_camera(true);
    ctx->camera.set_flag_thread_camera_alive(true);
    ctx->camera.thread_camera = boost::thread{ fn_camera };
    nlohmann::json j;
    j["prop"] = PROP_CALIBRATINGCAMERA;
    j["value"] = true;
    ctx->stremit(EV_PROPCHANGED, j.dump(), true);
    j["prop"] = PROP_DISPLAYVIDEO;
    j["value"] = true;
    ctx->stremit(EV_PROPCHANGED, j.dump(), true);
    ctx->stremit(EV_CAMERACALIBSTART, "", true);
}
}