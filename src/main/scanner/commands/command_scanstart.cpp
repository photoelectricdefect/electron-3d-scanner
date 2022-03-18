#include <commands/command_scanstart.hpp>
#include <commands/command_videocapturestart.hpp>
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

    command_scanstart::command_scanstart(scanner* ctx, int code) : command(ctx, code) {}

    void command_scanstart::execute()
    {
        command_videocapturestart comm_video_capture_start(ctx,COMM_VIDEOCAPTURESTART);
        comm_video_capture_start.execute();

        ctx->set_flag_scanning(true);
        ctx->camera.set_flag_thread_camera_alive(true);
        // using K = ctx->camera.camera_calibration.K;
        // using D = ctx->camera.camera_calibration.D;

        auto fn_camera = [ctx=ctx,K=ctx->camera.camera_calibration.K,D=ctx->camera.camera_calibration.D]()
        {
            //thread_video_alive

            // ctx->camera.clear_key_camera();
            // auto RT = ctx->sccalib.get_axis_rigid_body_transform();
            // auto R = RT.block<3, 3>(0, 0);
            // auto t = RT.block<3, 1>(0, 3);
            // std::cout << "RT: " << RT << std::endl;
            // std::cout << "R: " << R << std::endl;
            // std::cout << "t: " << t << std::endl;

            // auto D = ctx->camera.calib.D;
            auto K_opencv = K;
            Eigen::Matrix3d K_eigen=Eigen::Matrix3d::Identity();
            K_eigen<<K.ptr<double>(0)[0],K.ptr<double>(0)[1],K.ptr<double>(0)[2],
            K.ptr<double>(1)[0],K.ptr<double>(1)[1],K.ptr<double>(1)[2],
            K.ptr<double>(2)[0],K.ptr<double>(2)[1],K.ptr<double>(2)[2];

            Eigen::Vector3d rotation_axis_direction=ctx->scanner_calibration.rotation_axis_direction;
            Eigen::Vector3d rotation_axis_origin=ctx->scanner_calibration.rotation_axis_origin;
            Eigen::Hyperplane<double, 3> laser_plane=ctx->scanner_calibration.laser_plane;
            double axis_radius = ctx->scanner_calibration.rotation_axis_radius;
            double stepper_gear_ratio = ctx->scanner_calibration.stepper_gear_ratio;
            double rotation_resolution=ctx->scconfig.rotation_resolution;
            int nsteps = stepper_gear_ratio * 360 / rotation_resolution;

            std::vector<double> direction, source;
            std::vector<std::vector<double>> orbit_points, center_points;
            int npoints;
            ctx->scanner_calibration.load_points("axis_points.json", direction, source, orbit_points, center_points, npoints);

            std::vector<double> axis_source_vector, axis_direction_vector;
            axis_direction_vector.push_back(ctx->scanner_calibration.rotation_axis_direction(0));
            axis_direction_vector.push_back(ctx->scanner_calibration.rotation_axis_direction(1));
            axis_direction_vector.push_back(ctx->scanner_calibration.rotation_axis_direction(2));
            axis_source_vector.push_back(ctx->scanner_calibration.rotation_axis_origin(0));
            axis_source_vector.push_back(ctx->scanner_calibration.rotation_axis_origin(1));
            axis_source_vector.push_back(ctx->scanner_calibration.rotation_axis_origin(2));

            const int DELAY_ROTATION=2300;

            auto pattern_size = ctx->scanner_calibration.pattern_size;
            int patternh = ctx->scanner_calibration.pattern_size.height,
                patternw = ctx->scanner_calibration.pattern_size.width,
                squareh = ctx->scanner_calibration.square_size,
                squarew = ctx->scanner_calibration.square_size;
        
            while (ctx->camera.get_flag_thread_camera_alive())
            {
                ctx->camera.clear_message_thread_camera();
                nlohmann::json message;
                ctx->camera.get_message_thread_camera(message);
                std::string type=message["type"];

                if(!type.compare("keyup")) {
                    int keycode =message["keycode"].get<int>();

                    if (keycode == KEYCODE_C)
                    {
                        // double rot_angle_degrees = 10 * ctx->scconfig.rotation_resolution;
                        // bool solved = false;
                        // bool foundcorners = false;
                        // std::vector<cv::Point3d> world_board_pts;
                        // std::vector<Eigen::Vector3d> source_points, destination_points;
                        // // ctx->sccalib.rotation_axis_direction.normalize();
                        // nlohmann::json response;

                        // bool rr = false;
                        // ctx->controller.rotate(ctx->scconfig.rotation_direction, rot_angle_degrees,17000, response, 18000, rr);

                        // for (int i = 0; i < patternh; i++)
                        // {
                        //     for (int j = 0; j < patternw; j++)
                        //     {
                        //         world_board_pts.push_back(cv::Point3d(j * squarew, i * squareh, 0));
                        //     }
                        // }

                        // while (!solved || !foundcorners)
                        // {
                        //     solved = false;
                        //     foundcorners = false;
                        //     cv::Mat rvecs, tvec;
                        //     std::vector<cv::Point2f> img_pts;
                        //     cv::Mat imtmp_, imtmp, graytmp;
                        //     boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                        //     cap->read(imtmp_);
                        //     lock.unlock();

                        //     if (imtmp_.empty())
                        //         continue;

                        //     cv::undistort(imtmp_, imtmp, K_opencv, D);
                        //     cv::cvtColor(imtmp, graytmp, cv::COLOR_BGR2GRAY);
                        //     foundcorners = cv::findChessboardCorners(graytmp, pattern_size, img_pts);

                        //     if (!foundcorners)
                        //         continue;

                        //     solved = cv::solvePnP(world_board_pts, img_pts, ctx->camera.calib.K, ctx->camera.calib.D, rvecs, tvec);

                        //     if (!solved)
                        //         continue;

                        //     cv::Rodrigues(rvecs, rvecs);
                        //     // std::cout<<"rafter"<<rvecs<<std::endl;
                        //     Eigen::Matrix<double, 3, 4> T;
                        //     T << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                        //         *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                        //         *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);

                        //     for (size_t i = 0; i < world_board_pts.size(); i++)
                        //     {
                        //         Eigen::Vector4d board_point(world_board_pts[i].x, world_board_pts[i].y, 0, 1);
                        //         source_points.push_back(T * board_point);
                        //     }
                        // }

                        // bool err = false;
                        // ctx->controller.rotate(ctx->scconfig.rotation_direction, rot_angle_degrees,17000, response, 18000, err);
                        // solved = false;
                        // foundcorners = false;

                        // while (!solved || !foundcorners)
                        // {
                        //     solved = false;
                        //     foundcorners = false;
                        //     cv::Mat rvecs, tvec;
                        //     std::vector<cv::Point2f> img_pts;
                        //     cv::Mat imtmp_, imtmp, graytmp;
                        //     boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                        //     cap->read(imtmp_);
                        //     lock.unlock();

                        //     if (imtmp_.empty())
                        //         continue;

                        //     cv::undistort(imtmp_, imtmp, K_opencv, D);
                        //     cv::cvtColor(imtmp, graytmp, cv::COLOR_BGR2GRAY);
                        //     foundcorners = cv::findChessboardCorners(graytmp, pattern_size, img_pts);

                        //     if (!foundcorners)
                        //         continue;

                        //     solved = cv::solvePnP(world_board_pts, img_pts, ctx->camera.calib.K, ctx->camera.calib.D, rvecs, tvec);

                        //     if (!solved)
                        //         continue;

                        //     cv::Rodrigues(rvecs, rvecs);
                        //     // std::cout<<"rafter"<<rvecs<<std::endl;
                        //     Eigen::Matrix<double, 3, 4> T;
                        //     T << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                        //         *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                        //         *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);

                        //     for (size_t i = 0; i < world_board_pts.size(); i++)
                        //     {
                        //         Eigen::Vector4d board_point(world_board_pts[i].x, world_board_pts[i].y, 0, 1);
                        //         destination_points.push_back(T * board_point);
                        //     }
                        // }

                        // double angle = 2 * M_PI * (rot_angle_degrees / 360);
                        // Eigen::Matrix3d R_axis = Eigen::AngleAxisd(angle, ctx->sccalib.rotation_axis_direction).toRotationMatrix();
                        // // Eigen::Matrix4d T_axis=Eigen::Matrix4d::Zero();
                        // // T_axis.block<3,3>(0,0)=R_axis;
                        // // T_axis.block<3,1>(0,3)=Eigen::Vector3d::Zero();
                        // // T_axis(3,3)=1;
                        // // Eigen::Matrix4d T_ref=Eigen::Matrix4d::Zero();
                        // // T_ref.block<3,3>(0,0)=R.transpose();
                        // // T_ref.block<3,1>(0,3)=-R.transpose()*t;
                        // // T_ref(3,3)=1;

                        // test_transform_acuraccy(source_points, destination_points, R_axis, t);
                    }
                    else if (keycode == KEYCODE_SPACE)
                    {
                        nlohmann::json j;
                        j["type"] = "axis";
                        j["source"] = axis_source_vector;
                        j["direction"] = axis_direction_vector;
                        j["orbit_points"] = orbit_points;
                        j["center_points"] = center_points;
                        j["npoints"] = npoints;
                        ctx->stremit(EV_SCANDATA, j.dump(), true);

                        Eigen::Matrix4d T_align_axis=Eigen::Matrix4d::Identity();
                        Eigen::Vector3d z_axis=(-rotation_axis_origin).dot(rotation_axis_direction)*rotation_axis_direction+rotation_axis_origin;                    
                        z_axis.normalize();
                        Eigen::Vector3d x_axis=z_axis.cross(rotation_axis_direction);
                        x_axis.normalize();
                        Eigen::Matrix3d R_align_axis = Eigen::Matrix3d::Zero();
                        R_align_axis.col(0)=x_axis;
                        R_align_axis.col(1)=rotation_axis_direction;
                        R_align_axis.col(2)=z_axis;
                        T_align_axis.block<3, 3>(0, 0) = R_align_axis.transpose();

                        double angle_ratio=(rotation_resolution / (360 * stepper_gear_ratio));

                        for (size_t i = 0; i < nsteps && ctx->camera.get_flag_thread_camera_alive(); i++)
                        {
                            nlohmann::json message_intermittent;
                            bool message_intermittent_recieved=ctx->camera.try_get_message_thread_camera(message_intermittent);

                            if(message_intermittent_recieved) {
                                type=message_intermittent["type"];

                                if(!type.compare("keyup")) {
                                    int keycode=message_intermittent["keycode"].get<int>();
                                
                                    if (keycode == KEYCODE_SPACE)
                                    {
                                        break;
                                    }
                                }
                            }

                            bool err = false;
                            nlohmann::json response;
                            ctx->controller.set_laser(1, 800, response, 2000, err);

                            if (err)
                            {
                                ctx->stremit(EV_ERROR, "error turning on laser", true);
                                break;
                            }

                            cv::Mat imlaser, imnolaser;
                           
                            // boost::unique_lock<boost::mutex> lock_video_capture0(ctx->camera.mutex_video_capture);
                            // ctx->camera.video_capture.read(imlaser);
                            // lock_video_capture0.unlock();
                           
                            boost::unique_lock<boost::mutex> lock_video_capture0(ctx->camera.mutex_video_capture);
                                
                            if(!ctx->camera.video_capture.read(imlaser)) {
                                ctx->camera.notify_video_closed();
                                break;
                            }

                            lock_video_capture0.unlock();
                           
                            err = false;
                            ctx->controller.set_laser(0, 10, response, 2000, err);

                            if (err)
                            {
                                ctx->stremit(EV_ERROR, "error turning off laser", true);
                                continue;
                            }

                            boost::unique_lock<boost::mutex> lock_video_capture1(ctx->camera.mutex_video_capture);
                            // ctx->camera.video_capture.read(imnolaser);

                            if(!ctx->camera.video_capture.read(imnolaser)) {
                                ctx->camera.notify_video_closed();
                                break;
                            }

                            lock_video_capture1.unlock();

                            cv::Mat base_colors0[3];
                            cv::split(imlaser, base_colors0);
                            cv::Mat R0 = base_colors0[2];

                            laser_detector ldet(imlaser, imnolaser, 2, 12);
                            auto laser_image_points = ldet.detect(1);
                            std::vector<std::vector<double>> object_points(3);
                            std::vector<std::vector<int>> object_point_colors(3);
                            double angle = 2 * M_PI * i * angle_ratio;
                            Eigen::Matrix3d R_object_frame = Eigen::AngleAxisd(angle, rotation_axis_direction).toRotationMatrix();
                            Eigen::Matrix4d T_object_frame = Eigen::Matrix4d::Identity();
                            T_object_frame.block<3, 3>(0, 0) = R_object_frame.transpose();
                            T_object_frame.block<3, 1>(0, 3) = -R_object_frame.transpose() * rotation_axis_origin;
                            Eigen::Matrix4d T_object_frame_aligned=T_align_axis*T_object_frame;

                            for (size_t i = 0; i < laser_image_points.size(); i++)
                            {
                                Eigen::Vector3d laser_camera_point;
                                int x=laser_image_points[i][0];
                                int y=laser_image_points[i][1];
                                Eigen::Vector2d laser_image_point(x,y);
                                get_laser_camera_coordinates(laser_image_point, laser_plane, K_eigen, laser_camera_point);
                                Eigen::Vector4d laser_camera_point_homogeneous(laser_camera_point(0), laser_camera_point(1), laser_camera_point(2), 1);
                                auto object_point = T_object_frame_aligned * laser_camera_point_homogeneous;
                                Eigen::Vector3d d = laser_camera_point - rotation_axis_origin;
                                Eigen::Vector3d d_orthogonal = d - rotation_axis_direction * d.dot(rotation_axis_direction);

                                if (d_orthogonal.norm() > axis_radius)
                                    continue;

                                object_points[0].push_back(object_point(0));
                                object_points[1].push_back(object_point(1));
                                object_points[2].push_back(object_point(2));
                                cv::Vec3b color=imnolaser.ptr<cv::Vec3b>(y)[x];
                                object_point_colors[0].push_back(color(0));
                                object_point_colors[1].push_back(color(1));
                                object_point_colors[2].push_back(color(2));
                                R0.ptr<uint8_t>(y)[x] = 255;
                            }

                            // auto imupdate = [self,&R0]() {
                            //     boost::unique_lock<boost::mutex> lock(ctx->camera.mtx_video_alive);

                            //     if (ctx->camera.video_alive) {
                            //         uint8_t* data0;
                            //         auto len0 = cv_helpers::mat2buffer(R0, data0);
                            //         ctx->imemit(EV_DEBUGCAPTURE, data0, len0, true);
                            //     }
                            // };

                            // imupdate();

                            nlohmann::json j;
                            j["type"] = "points";
                            // j["npoints"] = laser_image_points.size();
                            j["positions"] = object_points;
                            j["colors"] = object_point_colors;
                            ctx->stremit(EV_SCANDATA, j.dump(), true);
                            err = false;
                            ctx->controller.rotate(ctx->scconfig.rotation_direction, 1,DELAY_ROTATION, response, DELAY_ROTATION+1000, err);
                        }
                    }

                }
            }
        };

        ctx->camera.thread_camera = boost::thread{fn_camera};
        nlohmann::json j;
        j["prop"] = PROP_SCANNING;
        j["value"] = true;
        ctx->stremit(EV_PROPCHANGED, j.dump(), true);
        ctx->stremit(EV_SCANSTART, "", true);
    }
}