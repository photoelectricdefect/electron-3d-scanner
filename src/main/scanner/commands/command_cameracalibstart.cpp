#include <commands/command_cameracalibstart.hpp>
#include <commands/command_cameracalibstop.hpp>
#include <commands/command_videostart.hpp>
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
    command_cameracalibstart::command_cameracalibstart(scanner& ctx, int code) : command(ctx, code) {};
    command_cameracalibstart::command_cameracalibstart(scanner& ctx, jcommand jcomm) : command(ctx, jcomm) {};

    void command_cameracalibstart::execute(std::shared_ptr<command> self) {
				ctx.camera.calibrating = true;
				ctx.camera.thread_alive = true;
				ctx.camera.video_alive = true;

                auto fn = [self, calib = ctx.camera.calib]() {
                    cv::VideoCapture cap;
	                cap.open(0);

	                if (!cap.isOpened())
                	{
		                std::cerr << "error opening camera" << std::endl;
		                return;
	                }
                    
                    int boardh = calib.board_size.height,
                        boardw = calib.board_size.width,
                        squareh = calib.square_size.height,
                        squarew = calib.square_size.width;
					std::vector<cv::Point3d> tmp;

                    for(int i = 0; i < boardh; i++) {
                        for(int j = 0; j < boardw; j++) {
                            tmp.push_back(cv::Point3d(j * squarew, i * squareh, 0));
                        }
                    }

                    std::vector<std::vector<cv::Point3d>> world_pts(calib.ncaps, tmp);
                    std::vector<std::vector<cv::Point2d>> img_pts;
                    cv::Mat frame, gray;
                    bool running = true, interrupted = false;
					int caps = 0;
                    self->ctx.camera.inputq.clear();

                    while(running) {
                        try {
                			boost::this_thread::sleep_for(boost::chrono::milliseconds(1000/FPS_30));   
							// nlohmann::json input;

							// auto readinput = [self, &input]() {
							// 	while(!self->ctx.camera.inputq.q.empty())  {
							// 		if(self->ctx.camera.inputq.q.size() == 1) input = self->ctx.camera.inputq.q.front();  

							// 		self->ctx.camera.inputq.q.pop();
							// 	}
							// };

							// self->ctx.camera.inputq.lock(readinput);
							auto input = self->ctx.camera.inputq.dequeue(); 
							int keycode = input["keycode"].get<int>();
							cap.read(frame);

		                    if (frame.empty())
		                    {
			                    std::cerr << "empty frame grabbed" << std::endl;
			                    continue;
		                    }
                            
                            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                            std::vector<cv::Point2d> pts;
                            bool found = cv::findChessboardCorners(gray, calib.board_size, pts);
                        
                            if(found) {
                                cv::cornerSubPix(gray, pts, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
                            
                                if(keycode == KEYCODE_SPACE) {
									img_pts.push_back(pts);
									caps++;

									if(caps >= calib.ncaps) continue;
								} 

                                cv::drawChessboardCorners(frame, calib.board_size, cv::Mat(pts), found);
                            }

							auto imupdate = [self, &frame]() {
								uint8_t* data;
								auto len = cv_helpers::mat2buffer(frame, data);
                    			if(self->ctx.camera.video_alive) self->ctx.imemit(EV_IMUPDATE, data, len, true);
                			};
                			
							self->ctx.lock(imupdate, self->ctx.camera.mtx_video_alive, true); 
                        }
                        catch(boost::thread_interrupted&) { running = false; interrupted = true; }
                    }

					if(!interrupted) {
						cv::Mat rvecs, tvec;
	                    cv::calibrateCamera(world_pts, img_pts, frame.size(), self->ctx.camera.calib.K, self->ctx.camera.calib.D, rvecs, tvec);
					}

					// auto resetvideo = [self]() {
					// 	self->ctx.commandq.q.push(std::shared_ptr<command>(new command_cameracalibstop(self->ctx, COMM_CAMERACALIBSTOP)));
					// 	self->ctx.commandq.q.push(std::shared_ptr<command>(new command_videostart(self->ctx, COMM_VIDEOSTART)));				
					// };

					// self->ctx.commandq.lock(resetvideo, self->ctx.commandq.mtx);

					self->ctx.commandq.enqueue(std::shared_ptr<command>(new command_cameracalibstop(self->ctx, COMM_CAMERACALIBSTOP)));
				};

                nlohmann::json j;
                j["prop"] = PROP_CALIBRATINGCAMERA;
                j["value"] = true;
                ctx.stremit(EV_PROPCHANGED, j.dump(), true); 
                j["prop"] = PROP_VIDEOALIVE;
                j["value"] = true;
                ctx.stremit(EV_PROPCHANGED, j.dump(), true);    
                ctx.stremit(EV_CAMERACALIBSTART, "", true);
                ctx.camera.thread_camera = boost::thread{fn};
    }
}