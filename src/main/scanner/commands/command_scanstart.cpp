#include <commands/command_scanstart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
command_scanstart::command_scanstart(scanner& ctx, int code)
    : command(ctx, code)
{
}
command_scanstart::command_scanstart(scanner& ctx, jcommand jcomm)
    : command(ctx, jcomm)
{
}

void command_scanstart::execute(std::shared_ptr<command> self)
{
    ctx.scanning = true;
    ctx.camera.video_alive = true;
    ctx.camera.thread_alive = true;

    auto fn = [self, conf = self->ctx.scconf]() {
        cv::VideoCapture cap;
        cap.open(0);

        if (!cap.isOpened()) {
            std::cerr << "error opening camera" << std::endl;
            return;
        }

        boost::mutex vcap_mtx;

        auto video = [self, &cap, &vcap_mtx]() {
            cv::Mat frame;
            bool running = true;

            while (running) {
                try {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                    vcap_mtx.lock();
                    cap.read(frame);
                    vcap_mtx.unlock();

                    if (frame.empty()) {
                        std::cerr << "empty frame grabbed" << std::endl;
                        continue;
                    }

                    auto imupdate = [self, &frame]() {
						uint8_t* data;
						auto len = cv_helpers::mat2buffer(frame, data);
                        if (self->ctx.camera.video_alive) self->ctx.imemit(EV_IMUPDATE, data, len, true);
                    };

                    self->ctx.lock(imupdate, self->ctx.camera.mtx_video_alive, true);
                }
                catch (boost::thread_interrupted&) {
                    running = false;
                }
            }
        };

        auto video_thread = boost::thread{ video };

        //capture image AFTER each step!!!
        int ncaps = conf.angle / conf.step_resolution, caps;
        bool running = true;

        if(!self->ctx.controller.serial_is_open()) self->ctx.controller.serial_is_open();

        while(running) {
            // boost::this_thread::interruption_point();
            // nlohmann::json response;

            // if(self->ctx.controller.serial_is_open()) {
            //     try {
            //         std::string comm = microcontroller::format("turn") +
            //             microcontroller::format("angle", scanconfig::STEP) + microcontroller::format("angle", scanconfig::CLOCKWISE);
            //         self->ctx.controller.serial_writeln(microcontroller::format("angle", 1));
            //         self->ctx.controller.serial_set_timeout(1000);
            //         response = self->ctx.controller.serial_readln();
            //     }
            //     catch(boost::system::system_error& e) {
            //                 //
            //     }
            // }
            // else {
            //             //TODO: report closed
            //             continue;
            // }             

            //TODO: do laser identification
        }
    };

    nlohmann::json j;
    j["prop"] = PROP_VIDEOALIVE;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    j["prop"] = PROP_SCANNING;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    ctx.stremit(EV_SCANSTART, "", true);
    ctx.camera.thread_camera = boost::thread{ fn };
}
}