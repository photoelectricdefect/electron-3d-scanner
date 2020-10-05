#include <commands/command_scannercalibstart.hpp>
#include <commands/command_scannercalibstop.hpp>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/thread.hpp>
#include <scannercalib.hpp>
#include <cameracalib.hpp>
#include <flags.hpp>
#include <json.hpp>
#include <helpers/cv_helpers.hpp>
#include <helpers/math_helpers.hpp>
#include <models/polygon.hpp>

namespace scanner {
command_scannercalibstart::command_scannercalibstart(scanner& ctx, int code)
    : command(ctx, code){};
command_scannercalibstart::command_scannercalibstart(scanner& ctx, jcommand jcomm)
    : command(ctx, jcomm){};

void ODR_plane_fit(const std::vector<Eigen::Vector3d>& laser_pts, Eigen::Hyperplane<double, 3> &plane)
{
	Eigen::Vector3d c0;

	for (int i = 0; i < laser_pts.size(); i++)
	{
		c0 += laser_pts[i];
	}

	c0 /= laser_pts.size();
	Eigen::MatrixXd A(laser_pts.size(), 3);

	for (int i = 0; i < laser_pts.size(); i++)
	{
		A << laser_pts[i] - c0;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.computeV();
	Eigen::Vector3d n = svd.matrixV().col(A.rows() - 1);
	n.normalize();
	plane = Eigen::Hyperplane<double, 3> (n, c0);
}

	void max_mean_ROI(const cv::Mat& img, cv::Rect& ROI, int win_size, int step) {
		ROI = cv::Rect(0, 0, 0, 0);
		int max_i = img.size().height - win_size,
		max_j = img.size().width - win_size;
		double max_mean_len = -FLT_MAX;

		for(int i = 0; i <= max_i; i += step) {
			for(int j = 0; j <= max_j; j += step) {
				cv::Rect roi(j, i, win_size, win_size);
				cv::Scalar mean = cv::mean(img(roi));
				cv::pow(mean, 2, mean);
				double mean_len = sqrt(cv::sum(mean)(0));

				if(mean_len > max_mean_len) {
					max_mean_len = mean_len;
					ROI = roi;
				}
			}
		}
	}

    bool find_laser(const cv::Mat& imlaser, const cv::Mat& imnolaser, const std::vector<Eigen::Vector2d>& board_corners, cv::Size board_size, line_segment& laser) {
        cv::Mat diff, mask_morphed, denoised, morphed, cross = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));  
		cv::Mat base_colors[3], R;
		diff = cv::abs(imlaser - imnolaser);
		cv::split(diff, base_colors);
        R = base_colors[2];
		cv::medianBlur(R, denoised, 3);
		const int threshold_low = 15;
        cv::threshold(denoised, mask_morphed, threshold_low, 255, cv::THRESH_BINARY);
		cv::morphologyEx(mask_morphed, mask_morphed, cv::MORPH_OPEN, cross);
		denoised.copyTo(morphed, mask_morphed);
		cv::Rect ROI;
		max_mean_ROI(morphed, ROI, 40, 15);
		cv::Mat mask, mask_roi;
		int autothreshold = cv::threshold(morphed(ROI), mask_roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		cv::threshold(morphed, mask, autothreshold, 255, cv::THRESH_BINARY);
		int npx = cv::countNonZero(mask);
	
		if(npx <= 0) return false;

		cv::Mat coords(cv::Size(2, npx), CV_64FC1);
		int idx = 0;

		for(int i = 0; i < mask.rows; i++) {
			for(int j = 0; j < mask.cols; j++) {
				if(cv_helpers::get_px(mask, j, i) == cv_helpers::MAX_INTENSITY) {
					coords.ptr<double>(idx)[0] = j;
					coords.ptr<double>(idx)[1] = i;
					idx++;
				}
			}
		}
	
		Eigen::Vector2d O;
		Eigen::MatrixXd V, D;
		cv_helpers::PCA(coords, V, D, O);
		double noiseratio, 
		axisratio = (V.row(0).norm() * D(0, 0)) / (V.row(1).norm() * D(1,1)), scalef = 22;

		if(morphed.size().width > morphed.size().height) noiseratio = (double)morphed.size().width / morphed.size().height;
		else noiseratio = (double)morphed.size().height / morphed.size().width;
	
		//PREVENT FALSE LASER CLASSIFICATION
		if(axisratio < scalef * noiseratio) return false;
	
		Eigen::Vector2d p = V.row(0);
		p.normalize();
		std::vector<line_segment> borders; 
        int idxB = board_size.width - 1, idxC = (board_size.height - 1) * board_size.width,  
		idxD = board_size.width * board_size.height - 1;
		borders.push_back(line_segment(board_corners[0], board_corners[idxB]));
		borders.push_back(line_segment(board_corners[0], board_corners[idxC]));
		borders.push_back(line_segment(board_corners[idxD], board_corners[idxB]));
	    borders.push_back(line_segment(board_corners[idxD], board_corners[idxC]));
        Eigen::Vector2d A, B, r = p;
        double umin = DBL_MAX;

        for(int i = 0; i < borders.size(); i++) {
            Eigen::Vector2d s = borders[i].b - borders[i].a;
	        double c = math_helpers::cross2D(r, s);

	        if (c != 0)
	        {
                Eigen::Vector2d pq = borders[i].a - O;
                double a = math_helpers::cross2D(pq, s),
                u = a / c;

                if(u >= 0 && u < umin) umin = u;
	        }
        }

        A = O + umin * r;
        umin = DBL_MAX;
        r = -p;

        for(int i = 0; i < borders.size(); i++) {
            Eigen::Vector2d s = borders[i].b - borders[i].a;
	        double c = math_helpers::cross2D(r, s);

	        if (c != 0)
	        {
                Eigen::Vector2d pq = borders[i].a - O;
                double a = math_helpers::cross2D(pq, s),
                u = a / c;

                if(u >= 0 && u < umin) umin = u;
	        }
        }

        B = O + umin * r;
		laser = line_segment(A, B);

		return true;
	}

//TODO: Fix the 100000 warnings Eigen throws
//TODO: Think about whether automatic calibration is necessary instead of manual by capturing images by user
void command_scannercalibstart::execute(std::shared_ptr<command> self)
{
    ctx.calibrating = true;
    ctx.camera.video_alive = true;
    ctx.camera.thread_alive = true;

    auto fn = [self, camcalib = ctx.camera.calib, sccalib = ctx.calib]() {
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
        
        std::vector<cv::Point3d> world_pts;
        int boardh = sccalib.board_size.height,
            boardw = sccalib.board_size.width,
            squareh = sccalib.square_size.height,
            squarew = sccalib.square_size.width;
        
        for (int i = 0; i < boardh; i++) {
            for (int j = 0; j < boardw; j++) {
                world_pts.push_back(cv::Point3d(j * squarew, i * squareh, 0));
            }
        }

        std::vector<Eigen::Vector3d> plane_pts;
        self->ctx.camera.inputq.clear();
        bool running = true;

        while (running) {
            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
                //nlohmann::json input;

                // auto readinput = [self, &input]() {
                //     while (!self->ctx.camera.inputq.q.empty()) {
                //         if (self->ctx.camera.inputq.q.size() == 1) input = self->ctx.camera.inputq.q.front();

                //         self->ctx.camera.inputq.q.pop();
                //     }
                // };

                // self->ctx.camera.inputq.lock(readinput);
                
                auto input = self->ctx.camera.inputq.dequeue(); 
                int keycode = input["keycode"].get<int>();

                if (keycode == KEYCODE_SPACE) {
                    if(!self->ctx.controller.serial_is_open()) self->ctx.controller.serial_is_open();

                    if(!self->ctx.controller.serial_is_open()) {
                        //TODO: report closed
                        continue;
                    }

                    cv::Mat imlaser, imnolaser, gray, rvecs, tvec;
                    std::vector<cv::Point2d> img_pts;
                    nlohmann::json response;

                    if(self->ctx.controller.serial_is_open()) {
                        try {
                            self->ctx.controller.serial_writeln(microcontroller::format("laseralive", 0));
                            self->ctx.controller.serial_set_timeout(1000);
                            response = self->ctx.controller.serial_readln();
                        }
                        catch(boost::system::system_error& e) {
                            //
                        }
                    }
                    else {
                        //TODO: report closed
                        continue;
                    } 

                    vcap_mtx.lock();
                    cap.read(imnolaser);
                    vcap_mtx.unlock();

                    if (imnolaser.empty())
                        continue;

                    bool foundcorners = false, solved = false;
                    cv::undistort(imnolaser, imnolaser, camcalib.K, camcalib.D);
                    cv::cvtColor(imnolaser, gray, cv::COLOR_BGR2GRAY);
                    foundcorners = cv::findChessboardCorners(gray, sccalib.board_size, img_pts);

                    if (foundcorners) solved = cv::solvePnP(world_pts, img_pts, camcalib.K, camcalib.D, rvecs, tvec);

                    if (!foundcorners || !solved)
                        continue;

                    if(self->ctx.controller.serial_is_open()) {
                        try {
                            self->ctx.controller.serial_writeln(microcontroller::format("laseralive", 1));
                            self->ctx.controller.serial_set_timeout(1000);
                            response = self->ctx.controller.serial_readln();
                        }
                        catch(boost::system::system_error& e) {
                            //TODO: handle error
                        }
                    }
                    else {
                        //TODO: report closed
                        continue;
                    } 

                    vcap_mtx.lock();
                    cap.read(imlaser);
                    vcap_mtx.unlock();

                    if (imlaser.empty()) continue;

                    cv::undistort(imlaser, imlaser, camcalib.K, camcalib.D);
                    // cv::cvtColor(imlaser, gray, cv::COLOR_BGR2GRAY);
                    // foundcorners = cv::findChessboardCorners(gray, sccalib.board_size, img_pts);

                    // if (foundcorners) solved = cv::solvePnP(world_pts, img_pts, camcalib.K, camcalib.D, rvecs, tvec);

                    // if (!foundcorners || !solved) continue;

                    std::vector<Eigen::Vector2d> corners;

                    for (auto& pt : img_pts) {
                        corners.push_back(Eigen::Vector2d(pt.x, pt.y));
                    }

                    Eigen::Vector2d a = corners[0],
                                    b = corners[sccalib.board_size.width - 1],
                                    c = corners[(sccalib.board_size.height - 1) * sccalib.board_size.width],
                                    d = corners[sccalib.board_size.width * sccalib.board_size.height - 1];
                    std::vector<Eigen::Vector2d> outer_corners;
                    outer_corners.push_back(a), outer_corners.push_back(b),
                        outer_corners.push_back(d), outer_corners.push_back(c);
                    polygon quad(outer_corners);
                    polygon poly(corners);
                    // rectangle frame = poly.frame();
                    rectangle frame = quad.frame();
                    polygon translated_poly = poly.translate(-frame.UL); //May be unnecessary check later
                    polygon translated_quad = quad.translate(-frame.UL);
                    cv::Mat imlaser_cropped, cut_mask, imlaser_cut,
                        imnolaser_cropped, /*imnolaser_mask,*/ imnolaser_cut, diff;
                    cv_helpers::crop(imlaser, imlaser_cropped, frame.UL, frame.BR);
                    cv_helpers::cut(cut_mask, imlaser_cropped.size(), translated_quad.sides);
                    imlaser_cropped.copyTo(imlaser_cut, cut_mask);
                    cv_helpers::crop(imnolaser, imnolaser_cropped, frame.UL, frame.BR);
                    //cut(imnolaser_mask, imnolaser_cropped.size(), translated_quad.sides);
                    imnolaser_cropped.copyTo(imnolaser_cut, cut_mask);
                    line_segment laser;
                    bool foundlaser = find_laser(imlaser_cut, imnolaser_cut, translated_poly.vertices, camcalib.board_size, laser);

                    if (foundlaser) {
                        line_segment translated_laser = laser.translate(frame.UL);
                        // cv::line(undist, cv::Point(translated_laser.a(0), translated_laser.a(1)),
                        //     cv::Point(translated_laser.b(0), translated_laser.b(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                        cv::Rodrigues(rvecs, rvecs);
                        Eigen::Matrix<double, 3, 4> RT;
                        RT << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                            *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                            *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);
                        Eigen::Matrix3d H;
                        H.col(0) = RT.col(0), H.col(1) = RT.col(1), H.col(2) = RT.col(3);

                        //FOR NOW USING ONLY HORIZONTAL LINES FOR CROSS RATIO

                        // int step_y = board_size.height / 3;

                        // for (int i = 0; i < board_size.width; i++) {
                        //     int B_idx = (board_size.height - 1) * board_size.width + i;
                        //     Eigen::Vector2f T(img_pts[i].x, img_pts[i].y),
                        //         B(img_pts[B_idx].x, img_pts[B_idx].y),
                        //         intersection;
                        //     bool intersects = intersection_line_segment(T, B,
                        //         translated_laser.a, translated_laser.b, intersection);

                        //     // cv::line(undist, cv::Point(T(0), T(1)),
                        //     //     cv::Point(B(0), B(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                        //     if (intersects) {
                        //         // cv::circle(undist, cv::Point(intersection(0), intersection(1)), 5, cv::Scalar(255, 0, 0));

                        //         int x = i * square_size.width, b_idx = i + step_y * board_size.width,
                        //             c_idx = i + 2 * step_y * board_size.width;
                        //         Eigen::Vector2f A(x, 0),
                        //             B(x, step_y * square_size.height),
                        //             C(x, 2 * step_y * square_size.height),
                        //             a(img_pts[i].x, img_pts[i].y),
                        //             b(img_pts[b_idx].x, img_pts[b_idx].y),
                        //             c(img_pts[c_idx].x, img_pts[c_idx].y);

                        //         float Q = cross_ratio(A(1), B(1), C(1),
                        //             0, (b - a).norm(), (c - a).norm(), (intersection - a).norm());
                        //         Eigen::Vector3d world_pt(x, Q, 1), camera_pt = H * world_pt;
                        //         // std::cout << "board coords: " << std::endl
                        //         //           << world_pt << std::endl;
                        //         plane_pts.push_back(camera_pt.cast<float>());

                        //         // std::cout << "world point vertical: " << std::endl
                        //         //           << world_pt << std::endl;
                        //         // std::cout << "camera point vertical: " << std::endl
                        //         //           << camera_pt << std::endl;

                        //         // if (ch == e)
                        //         //     plane_pts.push_back(camera_pt.cast<float>());
                        //     }
                        // }

                        int step_x = boardw / 3;

                        for (int i = 0; i < boardh; i++) {
                            int L_idx = i * boardw, R_idx = L_idx + boardw - 1;
                            Eigen::Vector2d L(img_pts[L_idx].x, img_pts[L_idx].y),
                                R(img_pts[R_idx].x, img_pts[R_idx].y),
                                intersection;
                            bool intersects = math_helpers::intersection_line_segment(L, R,
                                translated_laser.a, translated_laser.b, intersection);

                            // cv::line(undist, cv::Point(L(0), L(1)),
                            //     cv::Point(R(0), R(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                            if (intersects) {
                                // cv::circle(undist, cv::Point(intersection(0), intersection(1)), 5, cv::Scalar(255, 0, 0));

                                int y = i * squareh, a_idx = i * boardw, b_idx = a_idx + step_x,
                                    c_idx = b_idx + step_x;
                                Eigen::Vector2d A(0, y),
                                    B(step_x * squarew, y),
                                    C(2 * step_x * squarew, y),
                                    a(img_pts[a_idx].x, img_pts[a_idx].y),
                                    b(img_pts[b_idx].x, img_pts[b_idx].y),
                                    c(img_pts[c_idx].x, img_pts[c_idx].y);

                                double Q = math_helpers::cross_ratio(A(0), B(0), C(0),
                                    0, (b - a).norm(), (c - a).norm(), (intersection - a).norm());
                                Eigen::Vector3d hmg_world_pt(Q, y, 1), camera_pt = H * hmg_world_pt;

                                // std::cout << "world point horitontal: " << std::endl
                                //           << world_pt << std::endl;
                                // std::cout << "camera point horizontal: " << std::endl
                                //           << camera_pt << std::endl;

                                // if (ch == e)
                                plane_pts.push_back(camera_pt);
                            }
                        }

						uint8_t* data;
						auto len = cv_helpers::mat2buffer(imlaser, data);
                        self->ctx.imemit(EV_SCANNERCALIBIMAGECAPTURED, data, len, true);
                    }
                }
            }
            catch (boost::thread_interrupted&) {
                running = false;
			}
        }

		video_thread.interrupt();
		video_thread.join();
	    self->ctx.commandq.enqueue(std::shared_ptr<command>(new command_scannercalibstop(self->ctx, COMM_SCANNERCALIBSTOP)));
	};

    nlohmann::json j;
    j["prop"] = PROP_CALIBRATINGSCANNER;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    j["prop"] = PROP_VIDEOALIVE;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    ctx.stremit(EV_SCANNERCALIBSTART, "", true);
    ctx.camera.thread_camera = boost::thread{ fn };
}
}