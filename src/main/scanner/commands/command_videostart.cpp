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
                ctx.camera.video_alive = true;
                ctx.camera.thread_alive = true;

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

            auto imupdate = [self, &frame]() {
                if(self->ctx.camera.video_alive) self->ctx.imemit(EV_IMUPDATE, std::shared_ptr<std::string>(new std::string(cv_helpers::mat2base64str(frame))), true);
            };

            while(running) {
                try{             
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000/FPS_30));   
		        cap.read(frame);

		        if (frame.empty())
		        {
			        std::cerr << "empty frame grabbed" << std::endl;
			        continue;
		        }

                self->ctx.lock(imupdate, self->ctx.camera.mtx_video_alive, true);    
                }
                catch (boost::thread_interrupted&) { running = false; }
            }
        };

        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);    
        ctx.stremit(EV_VIDEOSTART, "", true);
        ctx.camera.thread_camera = boost::thread{fn};
        }
}