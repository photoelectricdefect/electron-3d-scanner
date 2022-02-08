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
#include <cameracalib.hpp>
#include <json.hpp>
#include <vector>

namespace scanner {
command_cameracalibstart::command_cameracalibstart(scanner& ctx, int code)
    : command(ctx, code){};

void command_cameracalibstart::execute(std::shared_ptr<command> self)
{
    ctx.camera.calibrating = true;
    ctx.camera.camera_alive = true;
    ctx.camera.video_alive = true;

    std::shared_ptr<cv::VideoCapture> cap(new cv::VideoCapture);
    int videoid=camera::get_videoid("USB 2.0 Camera: USB Camera");

    if(videoid==-1) {
        std::cerr<<"could not detect USB camera"<<std::endl;
    }

    cap->open(videoid);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, 1600);          
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1200);         

    if (!cap->isOpened()) {
        std::cerr << "error opening camera" << std::endl;
    }

    auto fncamera = [ self,cap]()
    {
        const bool release_object=true;
        int n_captures=self->ctx.camera.calib.n_captures;
        auto pattern_size=self->ctx.camera.calib.pattern_size;
        int patternh = pattern_size.height,
            patternw = pattern_size.width;
        double squareh = self->ctx.camera.calib.square_width,
            squarew = self->ctx.camera.calib.square_width;

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

        cv::Mat frame, gray;
        bool running = true, interrupted = false;
        int current_captures = 0;
        self->ctx.camera.clear_key_camera();
        // std::vector<std::vector<cv::Point2f>> image_board_points_history;
        int ncorners=patternh*patternw;

        while (running) {
            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                int keycode = self->ctx.camera.get_key_camera();                
                cap->read(frame);

                if (frame.empty()) {
                    std::cerr << "empty frame grabbed" << std::endl;
                    continue;
                }

                // const int history_len=5;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Point2f> image_points;
                bool found = cv::findChessboardCorners(gray, pattern_size, image_points,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

                if (found) {
                    cv::cornerSubPix(gray, image_points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
                    // image_board_points_history.push_back(image_points);
                    // std::vector<cv::Point2f> avg_image_board_points;

                    // if(image_board_points_history.size()>history_len) {
                    //     image_board_points_history.erase(image_board_points_history.begin());
                    // }

                    //     for(int i=0;i<ncorners;i++) {
                    //         float x=0,y=0; 

                    //         for(int j=0;j<image_board_points_history.size();j++) {
                    //             x+=image_board_points_history[j][i].x;
                    //             y+=image_board_points_history[j][i].y;
                    //         }

                    //         x/=image_board_points_history.size();
                    //         y/=image_board_points_history.size();
                    //         avg_image_board_points.push_back(cv::Point2f(x,y));
                    //     }


                    // cv::Mat rvecs, tvec;
                    // bool solved = cv::solvePnP(world_board_points[0],image_points,self->ctx.camera.calib.K,self->ctx.camera.calib.D,rvecs,tvec);
                    
                    // if(solved) {
                    //     cv::Rodrigues(rvecs, rvecs);
                    //     std::cout<<"rvecs:"<<rvecs<<std::endl;
                    //     std::cout<<"tvec:"<<tvec<<std::endl;
                    // }

                    if (keycode == KEYCODE_SPACE) {
                        image_board_points.push_back(image_points);
                        current_captures++;
                        // nlohmann::json j;
                        // j["prop"] = "cameracalibcapturecomplete";
                        // j["value"] = caps;
                        self->ctx.stremit(EV_CAMERACALIBCAPTURED, "", true);

                        if (current_captures >= n_captures)
                            running = false;
                    }

                    cv::drawChessboardCorners(frame, pattern_size, cv::Mat(image_points), found);
                }

                auto imupdate = [self, &frame]() {
                    boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                    if (self->ctx.camera.video_alive) {
                        uint8_t* data;
                        auto len = cv_helpers::mat2buffer(frame, data);
                        self->ctx.imemit(EV_IMUPDATE, data, len, true);
                    }
                };

                imupdate();
            }
            catch (boost::thread_interrupted&) {
                running = false;
                interrupted = true;
            }
        }

        if (!interrupted) {
            cv::Mat rvecs, tvec,K,D;
            std::vector<cv::Point3f> new_world_board_points;
            int i_fixed_point = -1;

            if (release_object){
                i_fixed_point = patternw - 1;
            }

            std::cout<<"rmsRO:"<<cv::calibrateCameraRO(world_board_points, image_board_points, frame.size(), i_fixed_point,K, D, rvecs, tvec,new_world_board_points)<<std::endl;
            self->ctx.camera.calib.K=K;
            self->ctx.camera.calib.D=D;
            std::cout<<"KCAM:"<<self->ctx.camera.calib.K<<std::endl;
            std::cout<<"DCAM:"<<self->ctx.camera.calib.D<<std::endl;
            self->ctx.camera.calib.save("cameracalib.json");
            boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_calibrated);
            self->ctx.camera.calibrated = true;
            lock.unlock();
            nlohmann::json j;
            j["prop"] = "cameracalibrated";
            j["value"] = true;
            self->ctx.stremit(EV_PROPCHANGED, j.dump(), true);

            self->ctx.invokeIO(std::shared_ptr<command>(new command_cameracalibstop(self->ctx, COMM_CAMERACALIBSTOP)));
            self->ctx.invokeIO(std::shared_ptr<command>(new command_videostart(self->ctx, COMM_VIDEOSTART)));
        }
    };

    nlohmann::json j;
    j["prop"] = PROP_CALIBRATINGCAMERA;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    j["prop"] = PROP_VIDEOALIVE;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    ctx.stremit(EV_CAMERACALIBSTART, "", true);
    ctx.camera.thread_camera = boost::thread{ fncamera };
}
}