#include <commands/command_scanstart.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <helpers/cv_helpers.hpp>
#include <helpers/laser_detector/laser_detector.hpp>
#include "json.hpp"
#include <limits>

namespace scanner
{

    void test_transform_acuraccy(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &destination, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        double error = 0;

        for (size_t i = 0; i < source.size(); i++)
        {
            // Eigen::Vector3d source_h(source[i](0),source[i](1),source[i](2),1),destination_h(destination[i](0),destination[i](1),destination[i](2),1);
            error += (R.transpose() * (destination[i] - t) + t - source[i]).norm();
            std::cout << "source: " << source[i] << std::endl;
            std::cout << "destination: " << destination[i] << std::endl;

            std::cout << "R*(source[i]-t)+t: " << R * (source[i] - t) + t << std::endl;
        }

        std::cout << "error: " << error << std::endl;
    }

    bool get_laser_camera_coordinates(const Eigen::Vector2d &laser_img_pt, const Eigen::Hyperplane<double, 3> &laser_plane, const Eigen::Matrix3d &K, Eigen::Vector3d &laser_camera_pt)
    {
        double u = laser_img_pt(0), v = laser_img_pt(1);
        Eigen::Vector3d Xi(u, v, 1);
        auto Xc = K.colPivHouseholderQr().solve(Xi);
        double denom = laser_plane.normal().dot(Xc);

        if (denom != 0)
            laser_camera_pt = -(laser_plane.coeffs()(3) / denom) * Xc;
        else
            return false;

        return true;
    }

    command_scanstart::command_scanstart(scanner &ctx, int code) : command(ctx, code)
    {
    }

    void command_scanstart::execute(std::shared_ptr<command> self)
    {
        ctx.scanning = true;
        ctx.camera.video_alive = true;
        ctx.camera.camera_alive = true;

        std::shared_ptr<boost::mutex> vcap_mtx(new boost::mutex);
        std::shared_ptr<cv::VideoCapture> cap(new cv::VideoCapture);
        cap->open(4);
        cap->set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1200);

        if (!cap->isOpened())
        {
            std::cerr << "error opening camera" << std::endl;
        }

        // TODO: Make settings object to save scan settings like resolution, etc...

        auto fnvideo = [self, cap, vcap_mtx]()
        {
            cv::Mat frame, undistorted;
            bool running = true;

            while (running)
            {
                try
                {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                    boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                    cap->read(frame);
                    lock.unlock();

                    if (frame.empty())
                    {
                        std::cerr << "empty frame grabbed" << std::endl;
                        continue;
                    }

                    cv::undistort(frame, undistorted, self->ctx.camera.calib.K, self->ctx.camera.calib.D);

                    auto imupdate = [self, /*&frame*/ &undistorted]()
                    {
                        boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                        if (self->ctx.camera.video_alive)
                        {
                            uint8_t *data;
                            auto len = cv_helpers::mat2buffer(undistorted /*frame*/, data);
                            self->ctx.imemit(EV_IMUPDATE, data, len, true);
                        }
                    };

                    imupdate();
                }
                catch (boost::thread_interrupted &)
                {
                    running = false;
                }
            }
        };

        auto fncamera = [self, cap, vcap_mtx]()
        {
            bool running = true;
            self->ctx.camera.clear_key_camera();
            self->ctx.sccalib.load("scanner_calib.json");
            auto RT = self->ctx.sccalib.get_axis_rigid_body_transform();
            auto R = RT.block<3, 3>(0, 0);
            auto t = RT.block<3, 1>(0, 3);
            std::cout << "RT: " << RT << std::endl;
            std::cout << "R: " << R << std::endl;
            std::cout << "t: " << t << std::endl;

            auto K = self->ctx.camera.calib.K;
            auto D = self->ctx.camera.calib.D;
            std::vector<double> direction, source;
            std::vector<std::vector<double>> orbit_points, center_points;
            int npoints;
            self->ctx.sccalib.load_points("axis_points.json", direction, source, orbit_points, center_points, npoints);

            std::vector<double> axis_source, axis_direction;
            axis_direction.push_back(self->ctx.sccalib.rotation_axis_direction(0));
            axis_direction.push_back(self->ctx.sccalib.rotation_axis_direction(1));
            axis_direction.push_back(self->ctx.sccalib.rotation_axis_direction(2));
            axis_source.push_back(self->ctx.sccalib.rotation_axis_origin(0));
            axis_source.push_back(self->ctx.sccalib.rotation_axis_origin(1));
            axis_source.push_back(self->ctx.sccalib.rotation_axis_origin(2));

            auto pattern_size = self->ctx.sccalib.pattern_size;
            int patternh = self->ctx.sccalib.pattern_size.height,
                patternw = self->ctx.sccalib.pattern_size.width,
                squareh = self->ctx.sccalib.square_size,
                squarew = self->ctx.sccalib.square_size;
            double stepper_gear_ratio = self->ctx.sccalib.stepper_gear_ratio;
            int nsteps = stepper_gear_ratio * 360 / self->ctx.scconfig.rotation_resolution;

            while (running)
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                int keycode = self->ctx.camera.get_key_camera();

                if (keycode == KEYCODE_C)
                {
                    double rot_angle_degrees = 10 * self->ctx.scconfig.rotation_resolution;
                    bool solved = false;
                    bool foundcorners = false;
                    std::vector<cv::Point3d> world_board_pts;
                    std::vector<Eigen::Vector3d> source_points, destination_points;
                    // self->ctx.sccalib.rotation_axis_direction.normalize();
                    nlohmann::json response;

                    bool rr = false;
                    self->ctx.controller.rotate(self->ctx.scconfig.rotation_direction, rot_angle_degrees, response, 18000, rr);

                    for (int i = 0; i < patternh; i++)
                    {
                        for (int j = 0; j < patternw; j++)
                        {
                            world_board_pts.push_back(cv::Point3d(j * squarew, i * squareh, 0));
                        }
                    }

                    while (!solved || !foundcorners)
                    {
                        solved = false;
                        foundcorners = false;
                        cv::Mat rvecs, tvec;
                        std::vector<cv::Point2f> img_pts;
                        cv::Mat imtmp_, imtmp, graytmp;
                        boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                        cap->read(imtmp_);
                        lock.unlock();

                        if (imtmp_.empty())
                            continue;

                        cv::undistort(imtmp_, imtmp, K, D);
                        cv::cvtColor(imtmp, graytmp, cv::COLOR_BGR2GRAY);
                        foundcorners = cv::findChessboardCorners(graytmp, pattern_size, img_pts);

                        if (!foundcorners)
                            continue;

                        solved = cv::solvePnP(world_board_pts, img_pts, self->ctx.camera.calib.K, self->ctx.camera.calib.D, rvecs, tvec);

                        if (!solved)
                            continue;

                        cv::Rodrigues(rvecs, rvecs);
                        // std::cout<<"rafter"<<rvecs<<std::endl;
                        Eigen::Matrix<double, 3, 4> T;
                        T << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                            *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                            *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);

                        for (size_t i = 0; i < world_board_pts.size(); i++)
                        {
                            Eigen::Vector4d board_point(world_board_pts[i].x, world_board_pts[i].y, 0, 1);
                            source_points.push_back(T * board_point);
                        }
                    }

                    bool err = false;
                    self->ctx.controller.rotate(self->ctx.scconfig.rotation_direction, rot_angle_degrees, response, 18000, err);
                    solved = false;
                    foundcorners = false;

                    while (!solved || !foundcorners)
                    {
                        solved = false;
                        foundcorners = false;
                        cv::Mat rvecs, tvec;
                        std::vector<cv::Point2f> img_pts;
                        cv::Mat imtmp_, imtmp, graytmp;
                        boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                        cap->read(imtmp_);
                        lock.unlock();

                        if (imtmp_.empty())
                            continue;

                        cv::undistort(imtmp_, imtmp, K, D);
                        cv::cvtColor(imtmp, graytmp, cv::COLOR_BGR2GRAY);
                        foundcorners = cv::findChessboardCorners(graytmp, pattern_size, img_pts);

                        if (!foundcorners)
                            continue;

                        solved = cv::solvePnP(world_board_pts, img_pts, self->ctx.camera.calib.K, self->ctx.camera.calib.D, rvecs, tvec);

                        if (!solved)
                            continue;

                        cv::Rodrigues(rvecs, rvecs);
                        // std::cout<<"rafter"<<rvecs<<std::endl;
                        Eigen::Matrix<double, 3, 4> T;
                        T << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                            *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                            *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);

                        for (size_t i = 0; i < world_board_pts.size(); i++)
                        {
                            Eigen::Vector4d board_point(world_board_pts[i].x, world_board_pts[i].y, 0, 1);
                            destination_points.push_back(T * board_point);
                        }
                    }

                    double angle = 2 * M_PI * (rot_angle_degrees / 360);
                    Eigen::Matrix3d R_axis = Eigen::AngleAxisd(angle, self->ctx.sccalib.rotation_axis_direction).toRotationMatrix();
                    // Eigen::Matrix4d T_axis=Eigen::Matrix4d::Zero();
                    // T_axis.block<3,3>(0,0)=R_axis;
                    // T_axis.block<3,1>(0,3)=Eigen::Vector3d::Zero();
                    // T_axis(3,3)=1;
                    // Eigen::Matrix4d T_ref=Eigen::Matrix4d::Zero();
                    // T_ref.block<3,3>(0,0)=R.transpose();
                    // T_ref.block<3,1>(0,3)=-R.transpose()*t;
                    // T_ref(3,3)=1;

                    test_transform_acuraccy(source_points, destination_points, R_axis, t);
                }
                else if (keycode == KEYCODE_SPACE)
                {
                    nlohmann::json j;
                    j["type"] = "axis";
                    j["source"] = axis_source;
                    j["direction"] = axis_direction;
                    j["orbit_points"] = orbit_points;
                    j["center_points"] = center_points;
                    j["npoints"] = npoints;
                    self->ctx.stremit(EV_SCANDATA, j.dump(), true);
                    // const Eigen::Vector4d anchor(1,0,0,1);
                    // Eigen::Matrix4d RT_anchor=Eigen::Matrix4d::Identity();

                    // DELETE LATER
                    //  continue;
                    Eigen::Vector3d axis_direction = self->ctx.sccalib.rotation_axis_direction;
                    axis_direction.normalize();
                    double axis_radius = self->ctx.sccalib.rotation_axis_radius;

                    for (size_t i = 0; i < nsteps; i++)
                    {
                        keycode = self->ctx.camera.get_key_camera();

                        if (keycode == KEYCODE_SPACE)
                            break;

                        bool err = false;
                        nlohmann::json response;
                        std::string cmsg = "laser;" + microcontroller::format("state", 1);
                        self->ctx.controller.send_message(cmsg, response, 2000, err);

                        if (err)
                        {
                            self->ctx.stremit(EV_ERROR, "", true);
                            continue;
                        }

                        cv::Mat imlaser, imnolaser;
                        boost::unique_lock<boost::mutex> lock0(*vcap_mtx);
                        cap->read(imlaser);
                        lock0.unlock();
                        err = false;
                        cmsg = "laser;" + microcontroller::format("state", 0);
                        self->ctx.controller.send_message(cmsg, response, 2000, err);

                        if (err)
                        {
                            self->ctx.stremit(EV_ERROR, "", true);
                            continue;
                        }

                        boost::unique_lock<boost::mutex> lock1(*vcap_mtx);
                        cap->read(imnolaser);
                        lock1.unlock();
                        cv::Mat base_colors0[3]; //,base_colors1[3],R0,R1;
                        cv::split(imlaser, base_colors0);
                        cv::Mat R0 = base_colors0[2];

                        laser_detector ldet(imlaser, imnolaser, 2, 12);
                        auto laser_image_points = ldet.detect(1);
                        std::vector<std::vector<double>> object_points(3);
                        double angle = 2 * M_PI * i * (self->ctx.scconfig.rotation_resolution / (360 * 1.55));
                        std::cout << "angle: " << angle << std::endl;
                        // std::cout<<"axis: "<<self->ctx.sccalib.rotation_axis_direction<<std::endl;
                        Eigen::Matrix3d R_axis = Eigen::AngleAxisd(angle, axis_direction).toRotationMatrix();
                        // R_axis=R_axis*R_AXISPREV;
                        // R_AXISPREV=R_axis;
                        std::cout << "RRT: " << R * R.transpose() << std::endl;

                        Eigen::Matrix4d T_inv = Eigen::Matrix4d::Zero();
                        T_inv.block<3, 3>(0, 0) = R_axis.transpose();
                        T_inv.block<3, 1>(0, 3) = -R_axis.transpose() * t;
                        T_inv(3, 3) = 1;

                        // std::cout<<"L ident: "<<RT_inverse*RT<<std::endl;
                        // std::cout<<"R ident: "<<RT*RT_inverse<<std::endl;

                        for (size_t i = 0; i < laser_image_points.size(); i++)
                        {
                            Eigen::Map<Eigen::Matrix3d> K((double *)self->ctx.camera.calib.K.data);
                            auto Kt = K.transpose();
                            Eigen::Vector3d laser_camera_point;
                            Eigen::Vector2d laser_image_point = Eigen::Vector2d(laser_image_points[i][0], laser_image_points[i][1]);
                            get_laser_camera_coordinates(laser_image_point, self->ctx.sccalib.laser_plane, Kt, laser_camera_point);
                            Eigen::Vector4d laser_camera_point_homogeneous(laser_camera_point(0), laser_camera_point(1), laser_camera_point(2), 1);
                            // auto rotated_laser_camera_point=R_axis*(laser_camera_point-t)+t;

                            // auto rotated_laser_camera_point=R_camera*homogeneous_laser_camera_point;
                            auto object_point = T_inv * laser_camera_point_homogeneous;

                            // auto object_camera_point=R_axis.transpose()*(laser_camera_point-t);
                            Eigen::Vector3d d = laser_camera_point - t;
                            Eigen::Vector3d d_orthogonal = d - axis_direction * d.dot(axis_direction);

                            if (d_orthogonal.norm() > axis_radius)
                                continue;

                            std::vector<double> tmp(3);
                            object_points[0].push_back(object_point(0));
                            object_points[1].push_back(object_point(1));
                            object_points[2].push_back(object_point(2));
                            R0.ptr<uint8_t>(laser_image_points[i][1])[laser_image_points[i][0]] = 255;
                        }

                        // auto imupdate = [self,&R0]() {
                        //     boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                        //     if (self->ctx.camera.video_alive) {
                        //         uint8_t* data0;
                        //         auto len0 = cv_helpers::mat2buffer(R0, data0);
                        //         self->ctx.imemit(EV_DEBUGCAPTURE, data0, len0, true);
                        //     }
                        // };

                        // imupdate();

                        nlohmann::json j;
                        j["type"] = "points";
                        j["data"] = object_points;
                        self->ctx.stremit(EV_SCANDATA, j.dump(), true);
                        err = false;
                        self->ctx.controller.rotate(self->ctx.scconfig.rotation_direction, 1, response, 8000, err);
                    }
                }
            }
        };

        nlohmann::json j;
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        self->ctx.camera.thread_video = boost::thread{fnvideo};
        j["prop"] = PROP_SCANNING;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        ctx.stremit(EV_SCANSTART, "", true);
        ctx.camera.thread_camera = boost::thread{fncamera};
    }
}