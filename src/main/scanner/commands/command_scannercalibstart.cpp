#include <commands/command_scannercalibstart.hpp>
#include <commands/command_scannercalibstop.hpp>
#include <helpers/cv_helpers.hpp>
#include <helpers/math_helpers.hpp>
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
#include <models/polygon.hpp>
#include <iostream>

namespace scanner {
command_scannercalibstart::command_scannercalibstart(scanner& ctx, int code)
    : command(ctx, code){};

Eigen::Vector3d plane_fit(const std::vector<Eigen::Vector3d>& laser_pts, Eigen::Hyperplane<double, 3>& plane)
{
    Eigen::Vector3d c0;

    for (int i = 0; i < laser_pts.size(); i++) {
        c0 += laser_pts[i];
    }

    c0 /= laser_pts.size();
    Eigen::MatrixXd A(laser_pts.size(), 3);

    for (int i = 0; i < laser_pts.size(); i++) {
        A.row(i)=(laser_pts[i] - c0).transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.computeV();
    Eigen::Vector3d n = svd.matrixV().col(A.cols() - 1);
    n.normalize();
    plane = Eigen::Hyperplane<double, 3>(n, c0);

    return c0;
}

void max_mean_ROI(const cv::Mat& img, cv::Rect& ROI, int win_size, int step)
{
    ROI = cv::Rect(0, 0, 0, 0);
    int max_i = img.size().height - win_size,
        max_j = img.size().width - win_size;
    double max_mean_len = -FLT_MAX;

    for (int i = 0; i <= max_i; i += step) {
        for (int j = 0; j <= max_j; j += step) {
            cv::Rect roi(j, i, win_size, win_size);
            cv::Scalar mean = cv::mean(img(roi));
            cv::pow(mean, 2, mean);
            double mean_len = sqrt(cv::sum(mean)(0));

            if (mean_len > max_mean_len) {
                max_mean_len = mean_len;
                ROI = roi;
            }
        }
    }
}

bool find_laser(cv::Mat& imlaser, cv::Mat& imnolaser, const std::vector<Eigen::Vector2d>& board_corners, cv::Size board_size, line_segment& laser)
{
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

    if (npx <= 0)
        return false;

    cv::Mat coords(cv::Size(2, npx), CV_64FC1);
    int idx = 0;

    for (int i = 0; i < mask.rows; i++) {
        for (int j = 0; j < mask.cols; j++) {
            if (cv_helpers::get_px(mask, j, i) == cv_helpers::MAX_INTENSITY) {
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
        axisratio = (V.row(0).norm() * D(0, 0)) / (V.row(1).norm() * D(1, 1)), scalef = 22;

    if (morphed.size().width > morphed.size().height)
        noiseratio = (double)morphed.size().width / morphed.size().height;
    else
        noiseratio = (double)morphed.size().height / morphed.size().width;

    //PREVENT FALSE LASER CLASSIFICATION
    if (axisratio < scalef * noiseratio)
        return false;

    Eigen::Vector2d p = V.row(0);
    p.normalize();
    std::vector<line_segment> borders;
    int idxB = board_size.width - 1, idxC = (board_size.height - 1) * board_size.width,
        idxD = board_size.width * board_size.height - 1;
    borders.push_back(line_segment(board_corners[0], board_corners[idxB]));
    borders.push_back(line_segment(board_corners[0], board_corners[idxC]));
    borders.push_back(line_segment(board_corners[idxD], board_corners[idxB]));
    borders.push_back(line_segment(board_corners[idxD], board_corners[idxC]));
    Eigen::Vector2d A,B;
    double diag=sqrt(pow(imlaser.cols,2)+pow(imlaser.rows,2)),minlen=DBL_MAX;

    for (int i = 0; i < borders.size(); i++) {
	    auto pq = borders[i].a - O;
		    auto r=(diag+1)*p;
		    auto s=borders[i].b - borders[i].a;
	    double a = math_helpers::cross2D(pq, r);
		    double b = math_helpers::cross2D(pq, s);
		    double c = math_helpers::cross2D(r, s);

	    if (c != 0)
	    {
		    double t = b / c,
			    u = a / c;

		    if (0 <= t && t <= 1 &&
			    0 <= u && u <= 1)
		    {
			    auto intersection = O + t * r;
		        auto vlen=(intersection-O).norm();

                if(vlen<minlen) {
                    minlen=vlen;
                    A=intersection;
                } 
            }
	}
    }

    minlen=DBL_MAX;

    for (int i = 0; i < borders.size(); i++) {
	    Eigen::Vector2d pq = borders[i].a - O,
		    r = -(diag+1)*p,
		    s = borders[i].b - borders[i].a;
	    double a = math_helpers::cross2D(pq, r),
		    b = math_helpers::cross2D(pq, s),
		    c = math_helpers::cross2D(r, s);

	    if (c != 0)
	    {
		    double t = b/c,
			    u = a/c;

		    if (0 <= t && t <= 1 &&
			    0 <= u && u <= 1)
		    {
			    auto intersection=O+t*r;
		        auto vlen=(intersection-O).norm();

                if(vlen<minlen) {
                    minlen=vlen;
                    B=intersection;
                } 

            }
	    }   
    }

    laser=line_segment(A,B);

    return true;
}

//TODO: Think about whether automatic calibration is necessary instead of manual by capturing images by user
void command_scannercalibstart::execute(std::shared_ptr<command> self)
{
    ctx.calibrating = true;
    ctx.camera.video_alive = true;
    ctx.camera.camera_alive = true;
    std::shared_ptr<boost::mutex> vcap_mtx(new boost::mutex);
    std::shared_ptr<cv::VideoCapture> cap(new cv::VideoCapture);
    cap->open(0);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, 1600);          
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1200);         

    if (!cap->isOpened()) {
        std::cerr << "error opening camera" << std::endl;
    }

    auto fnvideo = [self, cap, vcap_mtx]() {
        cv::Mat frame;
        bool running = true;

        while (running) {
            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                // cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);         
                cap->read(frame);
                lock.unlock();

                if (frame.empty()) {
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
            catch (boost::thread_interrupted&) {
                running = false;
            }
        }
    };

    auto fncamera = [ self,cap,vcap_mtx ]()
    {
        std::vector<cv::Point3d> world_pts;
        int boardh = self->ctx.calib.board_size.height,
            boardw = self->ctx.calib.board_size.width,
            squareh = self->ctx.calib.square_size.height,
            squarew = self->ctx.calib.square_size.width;

        for (int i = 0; i < boardh; i++) {
            for (int j = 0; j < boardw; j++) {
                world_pts.push_back(cv::Point3d(j * squarew, i * squareh, 0));
            }
        }

        std::vector<std::pair<int, Eigen::Vector3d> > plane_pts;
        bool running = true;
        self->ctx.camera.clear_key_camera();
        int captures = 0;

        while (running) {
            try {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                int keycode = self->ctx.camera.get_key_camera();

                if(keycode==KEYCODE_C) {
                    Eigen::Hyperplane<double,3> laser_plane(Eigen::Vector3d(1,1,1),1);
                    std::vector<Eigen::Vector3d> tmp;

                    for(auto pair:plane_pts) {
                        tmp.push_back(pair.second);
                    }

                    auto c0=plane_fit(tmp,laser_plane);

                    self->ctx.calib.laser_plane=laser_plane;
                    Eigen::Matrix<double,4,1> n=laser_plane.coeffs();

                    std::cout<<"n:"<<n<<std::endl;

                    std::vector<double> nvec,centroid;
                    centroid.push_back(c0(0)),centroid.push_back(c0(1)),centroid.push_back(c0(2));
                    nvec.push_back(n(0)),nvec.push_back(n(1)),nvec.push_back(n(2)),nvec.push_back(n(3));
                    nlohmann::json j;
                    j["type"] = "plane";
                    j["n"] = nvec;
                    j["centroid"] = centroid;
                    self->ctx.stremit(EV_SCANNERCALIBDATA,j.dump(),true);
                }

                if (keycode == KEYCODE_SPACE) {
                    bool err = false;
                    nlohmann::json response;
                    std::string msg="laser;" + microcontroller::format("state", 0);
                    self->ctx.controller.send_message(msg,response,500,err);

                    // std::cout<<response.dump()<<std::endl;

                    // try {
                    //     if (!self->ctx.controller.serial_is_open()) {
                    //         self->ctx.controller.serial_open();
                    //         boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
                    //     }
                            
                    //     boost::system::error_code ferr;
                    //     self->ctx.controller.serial_flush(microcontroller::flush_io, ferr);

                    //     if (ferr)
                    //         BOOST_THROW_EXCEPTION(std::runtime_error(ferr.message()));

                    //     self->ctx.controller.serial_write_string("laser;" + microcontroller::format("state", 0) + "\r");
                    //     self->ctx.controller.serial_set_timeout(500);
                    //     response=self->ctx.controller.serial_readln();

                    //     // std::cout<<response.dump()<<std::endl;
                    // }
                    // catch (boost::system::system_error& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }
                    // catch (timeout_exception& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }
                    // catch (std::exception& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }

                    if (err) {
                        self->ctx.stremit(EV_ERROR, "", true);
                        continue;
                    }

                    cv::Mat imlaser, imlaser_, imnolaser, imnolaser_, gray, rvecs, tvec;
                    std::vector<cv::Point2d> img_pts;
                    boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                    // cap->set(cv::CAP_PROP_FRAME_WIDTH, 3264);          
                    // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 2448);         
                    cap->read(imnolaser_);
                    // boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
                    // cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                    // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);         
                    lock.unlock();

                    if (imnolaser_.empty())
                        continue;

                    bool foundcorners = false, solved = false;
                    cv::undistort(imnolaser_, imnolaser, self->ctx.camera.calib.K, self->ctx.camera.calib.D);
                    cv::cvtColor(imnolaser, gray, cv::COLOR_BGR2GRAY);
                    foundcorners = cv::findChessboardCorners(gray,self->ctx.calib.board_size,img_pts);

                    if (foundcorners)
                        solved = cv::solvePnP(world_pts,img_pts,self->ctx.camera.calib.K,self->ctx.camera.calib.D,rvecs,tvec);

                    std::cout<<"foundcorners: "<<foundcorners<<", solved: "<<solved<<std::endl;

                    if (!foundcorners || !solved)
                        continue;
                        
                    err = false;

                    // try {
                    //     if (!self->ctx.controller.serial_is_open()) {
                    //         self->ctx.controller.serial_open();
                    //         boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
                    //     }
                            
                    //     boost::system::error_code ferr;
                    //     self->ctx.controller.serial_flush(microcontroller::flush_io, ferr);

                    //     if (ferr)
                    //         BOOST_THROW_EXCEPTION(std::runtime_error(ferr.message()));

                    //     self->ctx.controller.serial_write_string("laser;" + microcontroller::format("state", 1) + "\r");
                    //     self->ctx.controller.serial_set_timeout(500);
                    //     response = self->ctx.controller.serial_readln();
                    // }
                    // catch (boost::system::system_error& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }
                    // catch (timeout_exception& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }
                    // catch (std::exception& e) {
                    //     err = true;
                    //     std::cerr << boost::diagnostic_information(e);
                    // }

                    msg="laser;" + microcontroller::format("state", 1);
                    self->ctx.controller.send_message(msg,response,500,err);

                    if (err) {
                        self->ctx.stremit(EV_ERROR, "", true);
                        continue;
                    }

                    // std::cout<<"3"<<std::endl;
                    boost::unique_lock<boost::mutex> lock_(*vcap_mtx);
                    // cap->set(cv::CAP_PROP_FRAME_WIDTH, 3264);          
                    // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 2448);         
                    cap->read(imlaser_);
                    // boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
                    // cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);          
                    // cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);         
                    lock_.unlock();

                    msg="laser;" + microcontroller::format("state", 0);
                    self->ctx.controller.send_message(msg,response,500,err);

                    if (err) {
                        self->ctx.stremit(EV_ERROR, "", true);
                        continue;
                    }

                    // std::cout<<"4"<<std::endl;

                    if (imlaser_.empty())
                        continue;

                    cv::undistort(imlaser_, imlaser, self->ctx.camera.calib.K, self->ctx.camera.calib.D);
                    std::vector<Eigen::Vector2d> corners;

                    for (auto& pt : img_pts) {
                        corners.push_back(Eigen::Vector2d(pt.x, pt.y));
                    }

                    Eigen::Vector2d a = corners[0],
                                    b = corners[self->ctx.calib.board_size.width - 1],
                                    c = corners[(self->ctx.calib.board_size.height - 1) * self->ctx.calib.board_size.width],
                                    d = corners[self->ctx.calib.board_size.width * self->ctx.calib.board_size.height - 1];
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
                    
                    // translated_quad.print_sides();

                    cv_helpers::cut(cut_mask, imlaser_cropped.size(), translated_quad.sides);
                    imlaser_cropped.copyTo(imlaser_cut,cut_mask);
                    cv_helpers::crop(imnolaser,imnolaser_cropped,frame.UL,frame.BR);
                    //cut(imnolaser_mask, imnolaser_cropped.size(), translated_quad.sides);
                    imnolaser_cropped.copyTo(imnolaser_cut,cut_mask);
                    line_segment laser;
                    bool foundlaser=find_laser(imlaser_cut,imnolaser_cut,translated_poly.vertices,self->ctx.camera.calib.board_size,laser);

                    std::cout<<"foundlaser: "<<foundlaser<<std::endl;

                        // line_segment lsa=translated_quad.sides[0];
                        // line_segment lsb=translated_quad.sides[1];
                        // line_segment lsc=translated_quad.sides[2];
                        // line_segment lsd=translated_quad.sides[3];

                        // cv::line(out, cv::Point(lsa.a(0), lsa.a(1)),
                        //      cv::Point(lsa.b(0), lsa.b(1)),255,2, cv::LINE_AA);
                        // cv::line(out, cv::Point(lsb.a(0), lsb.a(1)),
                        //      cv::Point(lsb.b(0), lsb.b(1)),255,2, cv::LINE_AA);
                        // cv::line(out, cv::Point(lsc.a(0), lsc.a(1)),
                        //      cv::Point(lsc.b(0), lsc.b(1)),255,2, cv::LINE_AA);
                        // cv::line(out, cv::Point(lsd.a(0), lsd.a(1)),
                        //      cv::Point(lsd.b(0), lsd.b(1)),255,2, cv::LINE_AA);

                            // uint8_t* data0;
                            // uint8_t* data1;
                            // // auto len0 = cv_helpers::mat2buffer(imlaser_cut, data0);
                            // auto len1 = cv_helpers::mat2buffer(out, data1);
                            // // self->ctx.imemit(EV_DEBUGCAPTURE, data0, len0, true);
                            // self->ctx.imemit(EV_DEBUGCAPTURE, data1, len1, true);

                    if (foundlaser) {
                        captures++;
                        auto id = std::to_string(captures);
                        line_segment translated_laser = laser.translate(frame.UL);
                        cv::line(imlaser, cv::Point(translated_laser.a(0), translated_laser.a(1)),
                        cv::Point(translated_laser.b(0), translated_laser.b(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                        // uint8_t* data0;
                        uint8_t* data;
                            // auto len0 = cv_helpers::mat2buffer(imlaser_cut, data0);
                        auto len = cv_helpers::mat2buffer(imlaser, data);
                            // self->ctx.imemit(EV_DEBUGCAPTURE, data0, len0, true);
                        self->ctx.imemit(EV_DEBUGCAPTURE, data, len, true);

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

                            nlohmann::json j;
                                std::vector<double> xyz(camera_pt.data(), camera_pt.data() + camera_pt.rows() * camera_pt.cols());
                                j["type"] = "point";
                                j["xyz"]=xyz;
                                j["id"] = id;
                                self->ctx.stremit(EV_SCANNERCALIBDATA, j.dump(), true);
                                plane_pts.push_back(std::pair<int, Eigen::Vector3d>(captures, camera_pt));
                        }

                        //UNCOMMENT
                        // captures++;

                        // auto imupdate = [self,&imlaser,captures]() {
                        //     std::cout<<"fhhffh"<<std::endl;
                        //     cv::resize(imlaser, imlaser, cv::Size(150,200));
                        //     uint8_t * data;
                        //     auto len=cv_helpers::mat2buffer(imlaser, data);
                        //     if (self->ctx.camera.video_alive) self->ctx.imemit(EV_SCANNERCALIBCAPTURED, data, std::to_string(captures), len, true);
                        // };

                        // imupdate();
                    }

                    auto imupdate = [self, &imlaser, id]() {
                        boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                        if (self->ctx.camera.video_alive) {
                            cv::Mat thumbnail;
                            cv::resize(imlaser, thumbnail, cv::Size(213, 120));
                            uint8_t* data0;
                            uint8_t* data1;
                            auto len0 = cv_helpers::mat2buffer(thumbnail, data0);
                            auto len1 = cv_helpers::mat2buffer(imlaser, data1);
                            nlohmann::json j;
                            j["type"] = "image";
                            j["size"] = "thumbnail";
                            j["id"] = id;
                            self->ctx.imemit(EV_SCANNERCALIBDATA, data0, j.dump(), len0, true);
                            j["type"] = "image";
                            j["size"] = "large";
                            j["id"] = id;
                            self->ctx.imemit(EV_SCANNERCALIBDATA, data1, j.dump(), len1, true);
                        }
                    };

                    imupdate();
                    }
                }
            }
            catch (boost::thread_interrupted&) {
                running = false;
            }
        }

        // video_thread.interrupt();
        // video_thread.join();
        // self->ctx.commandq.enqueue(std::shared_ptr<command>(new command_scannercalibstop(self->ctx, COMM_SCANNERCALIBSTOP)));
    };

    nlohmann::json j;
    j["prop"] = PROP_CALIBRATINGSCANNER;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    j["prop"] = PROP_VIDEOALIVE;
    j["value"] = true;
    ctx.stremit(EV_PROPCHANGED, j.dump(), true);
    ctx.stremit(EV_SCANNERCALIBSTART, "", true);
    ctx.camera.thread_camera = boost::thread{ fncamera };
    ctx.camera.thread_video = boost::thread{ fnvideo };
}
}