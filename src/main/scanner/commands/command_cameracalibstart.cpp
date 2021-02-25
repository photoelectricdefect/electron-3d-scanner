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
#include <flags.hpp>
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
    cap->open(0);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);          
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);         

        if (!cap->isOpened()) {
            std::cerr << "error opening camera" << std::endl;
        }

    auto fncamera = [ self, calib = ctx.camera.calib,cap]()
    {
        // nlohmann::json j;
        // j["prop"] = "captures";
        // j["value"] = 0;
        // self->ctx.stremit(EV_PROPCHANGED, j.dump(), true);

        int boardh = calib.board_size.height,
            boardw = calib.board_size.width,
            squareh = calib.square_size.height,
            squarew = calib.square_size.width;
        std::vector<cv::Point3f> tmp;

        for (int i = 0; i < boardh; i++) {
            for (int j = 0; j < boardw; j++) {
                tmp.push_back(cv::Point3f(j * squarew, i * squareh, 0));
            }
        }

        std::vector<std::vector<cv::Point3f> > world_pts(calib.captures, tmp);
        std::vector<std::vector<cv::Point2f> > img_pts;
        cv::Mat frame, gray;
        bool running = true, interrupted = false;
        int caps = 0;
        self->ctx.camera.clear_key_camera();

        while (running) {
            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                int keycode = self->ctx.camera.get_key_camera();
                cap->read(frame);

                if (frame.empty()) {
                    std::cerr << "empty frame grabbed" << std::endl;
                    continue;
                }

                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                std::vector<cv::Point2f> pts;
                bool found = cv::findChessboardCorners(gray, calib.board_size, pts);

                if (found) {
                    cv::cornerSubPix(gray, pts, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

                    if (keycode == KEYCODE_SPACE) {
                        img_pts.push_back(pts);
                        caps++;
                        // nlohmann::json j;
                        // j["prop"] = "cameracalibcapturecomplete";
                        // j["value"] = caps;
                        self->ctx.stremit(EV_CAMERACALIBCAPTURED, "", true);

                        if (caps >= calib.captures)
                            running = false;
                    }

                    cv::drawChessboardCorners(frame, calib.board_size, cv::Mat(pts), found);
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
            cv::Mat rvecs, tvec;
            cv::calibrateCamera(world_pts, img_pts, frame.size(), self->ctx.camera.calib.K, self->ctx.camera.calib.D, rvecs, tvec);
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