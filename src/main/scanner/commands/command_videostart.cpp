#include <commands/command_videostart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <json.hpp>

namespace scanner {
    command_videostart::command_videostart(scanner& ctx, int code) : command(ctx, code) {}

            void command_videostart::execute(std::shared_ptr<command> self) {
                ctx.camera.video_alive = true;
                std::shared_ptr<cv::VideoCapture> cap(new cv::VideoCapture);
                cap->open(0);
                cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);         

	        if (!cap->isOpened())
	        {
		        std::cerr << "error opening camera" << std::endl;
	        }

                auto fn = [self,cap](){

	        cv::Mat frame;
	        bool running = true;

            while(running) {
                try{             
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000/FPS_30));   
		        cap->read(frame);

                std::cout << frame.size()<<std::endl;

		        if (frame.empty())
		        {
			        std::cerr << "empty frame grabbed" << std::endl;
			        continue;
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
                catch (boost::thread_interrupted&) { running = false; }
            }
        };

        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);    
        ctx.stremit(EV_VIDEOSTART, "", true);
        ctx.camera.thread_video = boost::thread{fn};
        }
}