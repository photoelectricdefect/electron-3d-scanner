#include <commands/command_videostart.hpp>

namespace scanner {
    command_videostart::command_videostart(scanner ctx, int code) : command(ctx, code) {}
    command_videostart::command_videostart(scanner ctx, jcommand jcomm) : command(ctx, jcomm) {}

            void command_videostart::execute() {
                _ctx.video_alive = true;

                auto fn = [](){
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
        
                _ctx.stremit(EV_IMUPDATE, cv_helpers::mat2base64str(frame), true);
                }
                catch (boost::thread_interrupted&) { running = false; }
            }
        };
    
        _ctx.thread_video = boost::thread{fn};
        _ctx.stremit(EV_VIDEOSTART, "", true);
        }
}