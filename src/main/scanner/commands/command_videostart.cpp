#include <commands/command_videostart.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>

namespace scanner {
    command_videostart::command_videostart(scanner& ctx, int code) : command(ctx, code) {}
    command_videostart::command_videostart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {}

            void command_videostart::execute(std::shared_ptr<command> self) {
                self->ctx.set_video_alive(true);

                auto fn = [self](){
            cv::VideoCapture cap;
	        cap.open(0);

	        if (!cap.isOpened())
	        {
		        std::cerr << "error opening camera" << std::endl;
		        return;
	        }

	        std::cout << "space = capture image" << std::endl;

	        cv::Mat frame;
	        bool running = true;

            while(running) {
		        try {
                boost::this_thread::interruption_point(); 

                char c = cv::waitKey((int)(1000 / FPS_60));
		        cap.read(frame);

		        if (frame.empty())
		        {
			        std::cerr << "empty frame grabbed" << std::endl;
			        continue;
		        }
        
                self->ctx.stremit(EV_IMUPDATE, cv_helpers::mat2base64str(frame), true);
                }
                catch (boost::thread_interrupted&) { running = false; }
            }
        };
    
        self->ctx.thread_video = boost::thread{fn};
        self->ctx.stremit(EV_VIDEOSTART, "", true);
        }
}