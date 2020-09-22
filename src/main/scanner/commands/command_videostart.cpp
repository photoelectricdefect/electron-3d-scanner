#include <commands/command_videostart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    command_videostart::command_videostart(scanner& ctx, int code) : command(ctx, code) {}
    command_videostart::command_videostart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

            void command_videostart::execute(std::shared_ptr<command> self) {
                ctx.video_alive = true;
                ctx.camera_alive = true;

                auto fn = [self](){
            cv::VideoCapture cap;
	        cap.open(0);

	        if (!cap.isOpened())
	        {
		        std::cerr << "error opening camera" << std::endl;
		        return;
	        }

	        cv::Mat frame;
	        bool running = true;

            while(running) {
                try{             
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000/FPS_30));   
		        cap.read(frame);

		        if (frame.empty())
		        {
			        std::cerr << "empty frame grabbed" << std::endl;
			        continue;
		        }

                auto readinput = [self, &frame]() {
                    if(self->ctx.video_alive) self->ctx.stremit(EV_IMUPDATE, cv_helpers::mat2base64str(frame), true);
                };

                self->ctx.lock(readinput, self->ctx.mtx_video_alive);    
                }
                catch (boost::thread_interrupted&) { running = false; }
            }
        };

        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);    
        ctx.stremit(EV_VIDEOSTART, "", true);
        ctx.thread_camera = boost::thread{fn};
        }
}