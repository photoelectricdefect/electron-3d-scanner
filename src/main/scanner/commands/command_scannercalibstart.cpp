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
#include <json.hpp>
#include <models/polygon.hpp>
#include <iostream>
#include "bradley_thresholding/summed-area-table/summed_area_table.hpp"
#include "bradley_thresholding/bradley_thresholding.hpp"
// #include "helpers/sphere_fitting.hpp"
#include "helpers/tls_line.hpp"
// #include "helpers/alglib/cpp/src/interpolation.h"
#include "helpers/sphere_fitting.hpp"
#include "helpers/hyperplane_fitting.hpp"

namespace scanner
{
    command_scannercalibstart::command_scannercalibstart(scanner &ctx, int code)
        : command(ctx, code){};

    typedef struct circle
    {
        Eigen::Vector3d center;
        double r;
    } circle;

    circle get_circle(const std::vector<Eigen::Vector3d> &points)
    {
        Eigen::Matrix3d W;
        Eigen::Vector3d b, x;
        double a = points[0].dot(points[0]);

        for (size_t i = 0; i < 3; i++)
        {
            W.row(i) = (points[0] - points[i + 1]).transpose();
            b(i) = a - points[i + 1].dot(points[i + 1]);
        }

        x = W.colPivHouseholderQr().solve(0.5 * b);
        double r = (points[0] - x).norm();

        return circle{x, r};
    }

    // bool get_rigid_body_transform(const std::vector<std::vector<Eigen::Vector3d>>& orbit_points,Eigen::Matrix4d& T,Eigen::Vector3d& direction,Eigen::Vector3d& origin) {
    //     int ncorners=orbit_points.size();
    //     int npairs=orbit_points[0].size()-1;

    //     typedef struct optimization_data {
    //         const std::vector<std::vector<Eigen::Vector3d>>* points;
    //     } optimization_data;

    //     optimization_data* opt_data=new optimization_data;
    //     opt_data->points=&orbit_points;

    //     int npairs_per_corner=orbit_points[0].size()-1;

    //     if(npairs_per_corner<4)
    //         return false;

    //     Eigen::Vector3d A=orbit_points[0][0],B=orbit_points[0][1],C=orbit_points[0][2],D=orbit_points[0][3];
    //     std::vector<Eigen::Vector3d> points(4);
    //     points[0]=A,points[1]=B,points[2]=C,points[3]=D;
    //     circle c=get_circle(points);
    //     Eigen::Vector3d u=B-A;
    //     Eigen::Vector3d v=C-A;
    //     Eigen::Vector3d q=u.cross(v);
    //     q.normalize();
    //     double theta=2*M_PI*(1.8/360);
    //     Eigen::Matrix3d R_init=Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose();

    //     std::vector<double> params(12);
    //     params[0]=R_init(0,0),params[1]=R_init(0,1),params[2]=R_init(0,2);
    //     params[3]=R_init(1,0),params[4]=R_init(1,1),params[5]=R_init(1,2);
    //     params[6]=R_init(2,0),params[7]=R_init(2,1),params[8]=R_init(2,2);
    //     params[9]=c.center(0),params[10]=c.center(1),params[11]=c.center(2);

    //     alglib::real_1d_array u_array;
    //     u_array.setlength(params.size());
    //     u_array.setcontent(params.size(),params.data());
    //     const int maxit=1000;
    //     alglib::ae_int_t maxits=maxit;
    //     alglib::ae_int_t info;
    //     alglib::minlmstate state;
    //     alglib::minlmreport rep;
    //     double diffstep = 1e-6;
    //     double tol = 1e-5;

    //     alglib::minlmcreatev(npairs_per_corner*orbit_points.size()*3+6,u_array, diffstep, state);

    //     //TODO: no harcoded bounds
    //     alglib::real_1d_array bndl = "[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1000,-1000,-1000]";
    //     alglib::real_1d_array bndu = "[+1,+1,+1,+1,+1,+1,+1,+1,+1,+1000,+1000,+1000]";
    //     alglib::minlmsetbc(state, bndl, bndu);

    //     //TODO: use jacobian, more accurate solution
    //     // alglib::minlmcreatevj(npairs_per_corner*orbit_points.size()*3+6,u_array, state);
    //     alglib::minlmsetcond(state, tol, maxits);

    //     auto fn=[](const alglib::real_1d_array &u,alglib::real_1d_array &fi,void *ptr)
    //     {
    //         std::cout<<u.tostring(6)<<std::endl;
    //         auto od = ((optimization_data*)ptr);
    //         Eigen::Matrix<double,3,3> R;
    //         R<<u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8];
    //         Eigen::Matrix<double,3,1> t;
    //         t<<u[9],u[10],u[11];

    //         const std::vector<std::vector<Eigen::Vector3d>>& points=*(od->points);
    //         int ncorners=points.size();
    //         int npairs=points[0].size()-1;

    //         for(int i=0;i<ncorners;i++) {
    //             for(int j=0;j<npairs;j++) {
    //                 auto X0=points[i][j];
    //                 auto X1=points[i][j+1];
    //                 int offset=(i*npairs+j)*3;
    //                 Eigen::Vector3d f=R*(X0-t)+t-X1;
    //                 fi[offset]=f(0);
    //                 fi[offset+1]=f(1);
    //                 fi[offset+2]=f(2);
    //             }
    //         }

    //         int offset=ncorners*npairs*3;
    //         Eigen::Matrix3d R_identity=R*R.transpose();
    //         fi[offset]=R_identity(0,0)-1;
    //         fi[offset+1]=R_identity(0,1);
    //         fi[offset+2]=R_identity(0,2);
    //         fi[offset+3]=R_identity(1,1)-1;
    //         fi[offset+4]=R_identity(1,2);
    //         fi[offset+5]=R_identity(2,2)-1;
    //     };

    //     auto fn_jac=[](const alglib::real_1d_array &u, alglib::real_1d_array &fi, alglib::real_2d_array &jac, void *ptr) {
    //         auto od = ((optimization_data*)ptr);
    //         Eigen::Matrix<double,3,3> R;
    //         R<<u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8];
    //         Eigen::Matrix<double,3,1> t;
    //         t<<u[9],u[10],u[11];

    //         Eigen::Matrix3d I=Eigen::Matrix3d::Identity();

    //         const std::vector<std::vector<Eigen::Vector3d>>& points=*(od->points);
    //         int ncorners=points.size();
    //         int npairs=points[0].size()-1;

    //         for(int i=0;i<ncorners;i++) {
    //             for(int j=0;j<npairs;j++) {
    //                 auto X0=points[i][j];
    //                 auto X1=points[i][j+1];
    //                 int offset=(i*npairs+j)*3;
    //                 Eigen::Vector3d f=R*(X0-t)+t-X1;
    //                 fi[offset]=f(0);
    //                 fi[offset+1]=f(1);
    //                 fi[offset+2]=f(2);

    //                 Eigen::Matrix3d C=-R+I;
    //                 Eigen::Vector3d b=X0-t;

    //                 jac[offset][0]=b(0);
    //                 jac[offset][1]=b(1);
    //                 jac[offset][2]=b(2);
    //                 jac[offset][9]=C(0,0);
    //                 jac[offset][10]=C(0,1);
    //                 jac[offset][11]=C(0,2);

    //                 int offset0=offset+1;
    //                 jac[offset0][3]=b(0);
    //                 jac[offset0][4]=b(1);
    //                 jac[offset0][5]=b(2);
    //                 jac[offset0][9]=C(1,0);
    //                 jac[offset0][10]=C(1,1);
    //                 jac[offset0][11]=C(1,2);

    //                 int offset1=offset+2;
    //                 jac[offset1][6]=b(0);
    //                 jac[offset1][7]=b(1);
    //                 jac[offset1][8]=b(2);
    //                 jac[offset1][9]=C(2,0);
    //                 jac[offset1][10]=C(2,1);
    //                 jac[offset1][11]=C(2,2);
    //             }
    //         }

    //         int offset=ncorners*npairs*3;
    //         Eigen::Matrix3d R_identity=R*R.transpose();

    //         Eigen::Vector3d r1=R.block<1,3>(0,0);
    //         Eigen::Vector3d r2=R.block<1,3>(1,0);
    //         Eigen::Vector3d r3=R.block<1,3>(2,0);

    //         jac[offset][0]=2*r1(0);
    //         jac[offset][1]=2*r1(1);
    //         jac[offset][2]=2*r1(2);
    //         fi[offset]=R_identity(0,0)-1;

    //         int offset0=offset+1;
    //         jac[offset0][0]=r2(0);
    //         jac[offset0][1]=r2(1);
    //         jac[offset0][2]=r2(2);
    //         jac[offset0][3]=r1(0);
    //         jac[offset0][4]=r1(1);
    //         jac[offset0][5]=r1(2);
    //         fi[offset0]=R_identity(0,1);

    //         int offset1=offset+2;
    //         jac[offset1][0]=r3(0);
    //         jac[offset1][1]=r3(1);
    //         jac[offset1][2]=r3(2);
    //         jac[offset1][6]=r1(0);
    //         jac[offset1][7]=r1(1);
    //         jac[offset1][8]=r1(2);
    //         fi[offset1]=R_identity(0,2);

    //         int offset2=offset+3;
    //         jac[offset2][3]=2*r2(0);
    //         jac[offset2][4]=2*r2(1);
    //         jac[offset2][5]=2*r2(2);
    //         fi[offset2]=R_identity(1,1)-1;

    //         int offset3=offset+4;
    //         jac[offset3][3]=r3(0);
    //         jac[offset3][4]=r3(1);
    //         jac[offset3][5]=r3(2);
    //         jac[offset3][6]=r2(0);
    //         jac[offset3][7]=r2(1);
    //         jac[offset3][8]=r2(2);
    //         fi[offset3]=R_identity(1,2);

    //         int offset4=offset+5;
    //         jac[offset4][6]=2*r3(0);
    //         jac[offset4][7]=2*r3(1);
    //         jac[offset4][8]=2*r3(2);
    //         fi[offset4]=R_identity(2,2)-1;
    //     };

    //     // alglib::minlmoptimize(state, fn,fn_jac,NULL, opt_data);
    //     alglib::minlmoptimize(state, fn,NULL, opt_data);
    //     alglib::minlmresults(state, u_array, rep);
    //     T=Eigen::Matrix4d::Identity();
    //     T(0,0)=u_array[0],T(0,1)=u_array[1],T(0,2)=u_array[2];
    //     T(1,0)=u_array[3],T(1,1)=u_array[4],T(1,2)=u_array[5];
    //     T(2,0)=u_array[6],T(2,1)=u_array[7],T(2,2)=u_array[8];
    //     origin(0)=u_array[9],origin(1)=u_array[10],origin(2)=u_array[11];
    //     T.block<3,1>(0,3)=origin;
    //     // std::cout<<"Rz: "<<opt_data->Rz<<std::endl;
    //     std::cout<<"R_init: "<<R_init<<std::endl;
    //     std::cout<<"T: "<<T<<std::endl;
    //     std::cout<<"origin: "<<origin<<std::endl;

    //     //dd

    //     //dd

    //     // Eigen::MatrixXd x=A.colPivHouseholderQr().solve(b);

    //     // Eigen::Matrix3d R_,R;
    //     // R_<<x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0),x(6,0),x(7,0),x(8,0);

    //     Eigen::Matrix3d R=T.block<3,3>(0,0);
    //     // Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(R);
    //     // Eigen::Matrix3d householderR=qr.matrixR();
    //     // Eigen::Matrix3d S=Eigen::Matrix3d::Zero();

    //     // for(int i=0;i<3;i++) {
    //     //     if(householderR(i,i)>0)
    //     //         S(i,i)=1;
    //     //     else if(householderR(i,i)<0)
    //     //         S(i,i)=-1;
    //     // }

    //     // // // std::cout<<"S:"<<S<<std::endl;

    //     // R=qr.matrixQ()*S;
    //     // T.block<3,3>(0,0)=R;

    //     // RT=Eigen::Matrix4d::Identity();
    //     // RT.block<3,3>(0,0)=R;
    //     // origin=x.block<3,1>(9,0);
    //     // RT.block<3,1>(0,3)=origin;
    //     // std::cout<<"RT:"<<RT<<std::endl;
    //     // std::cout<<"origin:"<<origin<<std::endl;

    //     double gamma=acos(0.5*(R.trace()-1));

    //     if(gamma<1e-9) {
    //         return false;
    //     }
    //     else if(M_PI-gamma<1e-9) {
    //         //TODO: https://www.geometrictools.com/Documentation/RotationRepresentations.pdf
    //         return false;
    //     }

    //     direction=Eigen::Vector3d(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1));
    //     direction.normalize();

    //     return true;
    // }

    void get_rotation_axis(std::vector<std::vector<Eigen::Vector3d>> orbit_points, Eigen::Vector3d &direction, Eigen::Vector3d &source, std::vector<std::vector<double>> &center_points)
    {
        const int maxit = 1000;
        const double tol = 1e-6;
        std::vector<Eigen::Vector3d> axis_points;

        for (int i = 0; i < orbit_points.size(); i++)
        {
            std::vector<double> X(orbit_points[i].size() * 3);

            for (int j = 0; j < orbit_points[i].size(); j++)
            {
                int offset = 3 * j;
                auto v = orbit_points[i][j];
                X[offset] = v(0);
                X[offset + 1] = v(1);
                X[offset + 2] = v(2);
            }

            // for(int j=0;j<X.size();j++) {
            //     std::cout<<X[j]<<",";
            // }

            // std::cout<<std::endl;

            std::vector<double> u0;
            std::vector<double> origin(3);
            Eigen::Vector3d A(X[0], X[1], X[2]), B(X[3], X[4], X[5]), C(X[6], X[7], X[8]), D(X[9], X[10], X[11]);
            std::vector<Eigen::Vector3d> points(4);
            points[0] = A, points[1] = B, points[2] = C, points[3] = D;
            circle c = get_circle(points);
            origin[0] = c.center(0);
            origin[1] = c.center(1);
            origin[2] = c.center(2);
            // Eigen::Vector3d u=B-A;
            // Eigen::Vector3d v=C-A;
            // Eigen::Vector3d q=u.cross(v);
            // q.normalize();
            // direction_[0]=q(0);
            // direction_[1]=q(1);
            // direction_[2]=q(2);
            double r = c.r;
            alglib::lsfitreport rep;
            sphere_fitting::fit_lev_marq(X, 3, origin, r, rep, maxit, tol);
            axis_points.push_back(Eigen::Vector3d(origin[0], origin[1], origin[2]));
        }

        // Eigen::Vector3d source,direction;
        tls_line::fit_eigen(axis_points, source, direction);
        center_points.push_back(std::vector<double>());
        center_points.push_back(std::vector<double>());
        center_points.push_back(std::vector<double>());
        for (size_t i = 0; i < axis_points.size(); i++)
        {
            center_points[0].push_back(axis_points[i](0));
            center_points[1].push_back(axis_points[i](1));
            center_points[2].push_back(axis_points[i](2));
        }

        // Eigen::Vector3d y_axis=direction;
        // Eigen::Vector3d x_axis(y_axis(1),-y_axis(0),0);
        // Eigen::Vector3d z_axis=y_axis.cross(x_axis);
        // x_axis.normalize(),y_axis.normalize(),z_axis.normalize();
        // Eigen::Matrix4d RT;
        // RT.block<1,3>(3,0).setZero();
        // RT.block<3,1>(0,0)=x_axis;
        // RT.block<3,1>(0,1)=y_axis;
        // RT.block<3,1>(0,2)=z_axis;
        // RT.block<3,1>(0,3)=source;
        // RT(3,3)=1;
        // std::cout<<"RT: "<<RT<<std::endl;

        // return RT;
    }

    // Eigen::Vector3d plane_fit_orthogonalEigen(const std::vector<Eigen::Vector3d>& laser_pts, Eigen::Hyperplane<double, 3>& plane)
    // {
    //     Eigen::MatrixXd A(3,laser_pts.size());

    //     for (int i = 0; i < laser_pts.size(); i++) {
    //         A.col(i) = laser_pts[i];
    //     }

    //     Eigen::Vector3d c0 = A.rowwise().mean();

    //     for (int i = 0; i < laser_pts.size(); i++) {
    //         A.col(i)-=c0;
    //     }

    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    //     Eigen::Matrix3d AAT = A*A.transpose();
    //     es.compute(AAT);
    //     Eigen::Vector3d n = es.eigenvectors().col(0);
    //     n.normalize();
    //     plane = Eigen::Hyperplane<double, 3>(n, c0);

    //     return c0;
    // }

    // Eigen::Vector3d plane_fit_orthogonalSVD(const std::vector<Eigen::Vector3d>& laser_pts, Eigen::Hyperplane<double, 3>& plane)
    // {
    //     Eigen::MatrixXd A(3,laser_pts.size());

    //     for (int i = 0; i < laser_pts.size(); i++) {
    //         A.col(i) = laser_pts[i];
    //     }

    //     Eigen::Vector3d c0 = A.rowwise().mean();

    //     for (int i = 0; i < laser_pts.size(); i++) {
    //         A.col(i)-=c0;
    //     }

    //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV | Eigen::ComputeThinU);
    //     svd.computeV();
    //     Eigen::Vector3d n = svd.matrixU().col(A.rows() - 1);
    //     n.normalize();
    //     plane = Eigen::Hyperplane<double, 3>(n, c0);

    //     return c0;
    // }

    cv::Rect max_mean_ROI(const cv::Mat &im, int winr)
    {
        std::vector<std::vector<uint8_t>> imvec(im.rows);

        for (size_t i = 0; i < im.rows; i++)
        {
            imvec[i] = std::vector<uint8_t>(im.ptr<uint8_t>(i), im.ptr<uint8_t>(i) + im.cols);
        }

        summed_area_table<uint8_t, long> sat(imvec);
        double max_mean = -1;
        cv::Rect ROI(-1, -1, -1, -1);

        for (size_t i = 0; i < im.rows; i++)
        {
            for (size_t j = 0; j < im.cols; j++)
            {
                double mean = sat.mean(j, i, winr);

                if (mean > max_mean)
                {
                    max_mean = mean;
                    int ulx = j - winr, uly = i - winr, brx = j + winr, bry = i + winr;
                    cv::Point UL(ulx > 0 ? ulx : 0, uly > 0 ? uly : 0), BR(brx < im.cols ? brx : im.cols - 1, bry < im.rows ? bry : im.rows - 1);
                    ROI = cv::Rect(UL, BR);
                }
            }
        }

        return ROI;
    }

    bool find_laser_line(std::shared_ptr<command> self, cv::Mat &imlaser, cv::Mat &imnolaser, const std::vector<Eigen::Vector2d> &image_board_points, cv::Size pattern_size, line_segment &laser_line)
    {
        cv::Mat diff, mask_blurred, mask, denoised, blurred, cross = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::Mat base_colors[3], R;
        diff = cv::abs(imlaser - imnolaser);
        cv::split(diff, base_colors);
        R = base_colors[2];
        cv::medianBlur(R, blurred, 3);
        const int threshold_low = 15;
        cv::threshold(blurred, mask_blurred, threshold_low, 255, cv::THRESH_BINARY);
        cv::morphologyEx(mask_blurred, mask_blurred, cv::MORPH_OPEN, cross);
        blurred.copyTo(denoised, mask_blurred);
        cv::Mat black_squares = cv::Mat::zeros(denoised.size(), CV_8UC1), white_squares = cv::Mat::zeros(denoised.size(), CV_8UC1);

        const auto cross_2D = [](const Eigen::Vector2d &u, const Eigen::Vector2d &v)
        {
            return u(0) * v(1) - v(0) * u(1);
        };

        std::vector<line_segment> pattern_borders;
        int idxB = pattern_size.width - 1, idxC = (pattern_size.height - 1) * pattern_size.width,
            idxD = pattern_size.width * pattern_size.height - 1;
        pattern_borders.push_back(line_segment(image_board_points[0], image_board_points[idxB]));
        pattern_borders.push_back(line_segment(image_board_points[0], image_board_points[idxC]));
        pattern_borders.push_back(line_segment(image_board_points[idxD], image_board_points[idxB]));
        pattern_borders.push_back(line_segment(image_board_points[idxD], image_board_points[idxC]));

        for (int x = 0; x < denoised.cols; x++)
        {
            for (int y = 0; y < denoised.rows; y++)
            {
                Eigen::Vector2d w(x, y);

                if (cv_helpers::inside_polygon(w, pattern_borders))
                {
                    int v_idx = -1, h_idx = -1;
                    line_segment a = pattern_borders[0], b = pattern_borders[1];
                    Eigen::Vector2d AB = a.b - a.a, AC = b.b - b.a;

                    for (int i = 0; i < pattern_size.height; i++)
                    {
                        int L_idx = i * pattern_size.width, R_idx = L_idx + pattern_size.width - 1;
                        Eigen::Vector2d L = image_board_points[L_idx], R = image_board_points[R_idx];

                        Eigen::Vector2d q = R - L;
                        q.normalize();
                        Eigen::Vector2d z = w - L;
                        Eigen::Vector2d z_orthogonal = z - z.dot(q) * q;

                        if (AC.dot(z_orthogonal) >= 0)
                        {
                            v_idx++;
                        }
                    }

                    for (int i = 0; i < pattern_size.width; i++)
                    {
                        int U_idx = i, B_idx = U_idx + (pattern_size.height - 1) * pattern_size.width;
                        Eigen::Vector2d U = image_board_points[U_idx], B = image_board_points[B_idx];

                        Eigen::Vector2d q = B - U;
                        q.normalize();
                        Eigen::Vector2d z = w - U;
                        Eigen::Vector2d z_orthogonal = z - z.dot(q) * q;

                        if (AB.dot(z_orthogonal) >= 0)
                        {
                            h_idx++;
                        }
                    }

                    // double min_orthogonal_distance=DBL_MAX;
                    // std::vector<line_segment> square_borders;
                    // int E_idx=v_idx*pattern_size.width+h_idx,F_idx=E_idx+1,
                    // G_idx=F_idx+pattern_size.width,H_idx=E_idx+pattern_size.width;
                    // square_borders.push_back(line_segment(image_board_points[E_idx],image_board_points[F_idx]));
                    // square_borders.push_back(line_segment(image_board_points[F_idx],image_board_points[G_idx]));
                    // square_borders.push_back(line_segment(image_board_points[G_idx],image_board_points[H_idx]));
                    // square_borders.push_back(line_segment(image_board_points[H_idx],image_board_points[E_idx]));

                    // for(int i=0;i<square_borders.size();i++) {
                    //     Eigen::Vector2d q=square_borders[i].b-square_borders[i].a;
                    //     q.normalize();
                    //     Eigen::Vector2d z=w-square_borders[i].a;
                    //     Eigen::Vector2d z_orthogonal=z-z.dot(q)*q;
                    //     double distance=z_orthogonal.norm();

                    //     if(distance<min_orthogonal_distance) {
                    //         min_orthogonal_distance=distance;
                    //     }
                    // }

                    if ((v_idx + h_idx) % 2 == 0)
                    {
                        black_squares.ptr<uint8_t>(y)[x] = denoised.ptr<uint8_t>(y)[x];
                    }
                    else
                    {
                        white_squares.ptr<uint8_t>(y)[x] = denoised.ptr<uint8_t>(y)[x];
                    }

                    // if(min_orthogonal_distance>border_padding) {
                    //     if((v_idx+h_idx)%2==0) {
                    //         black_squares.ptr<uint8_t>(y)[x]=denoised.ptr<uint8_t>(y)[x];
                    //     }
                    //     else {
                    //         white_squares.ptr<uint8_t>(y)[x]=denoised.ptr<uint8_t>(y)[x];
                    //     }
                    // }
                    // else {
                    //         black_squares.ptr<uint8_t>(y)[x]=255;
                    //         white_squares.ptr<uint8_t>(y)[x]=255;
                    // }
                }
            }
        }

        auto ROI_black = max_mean_ROI(black_squares, 10);
        cv::Mat tmp_black, mask_black;
        int autothreshold = cv::threshold(black_squares(ROI_black), tmp_black, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(black_squares, mask_black, autothreshold, 255, cv::THRESH_BINARY);

        auto ROI_white = max_mean_ROI(white_squares, 10);
        cv::Mat tmp_white, mask_white;
        autothreshold = cv::threshold(white_squares(ROI_white), tmp_white, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(white_squares, mask_white, autothreshold, 255, cv::THRESH_BINARY);

        // something is off, check and fix
        auto mean_black = cv::mean(black_squares(ROI_black));
        auto mean_white = cv::mean(white_squares(ROI_white));
        std::cout << "mean_black: " << cv::norm(mean_black) << std::endl;
        std::cout << "mean_white: " << cv::norm(mean_white) << std::endl;

        mask = cv::norm(mean_black) >= cv::norm(mean_white) ? mask_black : mask_white;

        // auto imupdate = [self,&mask,&denoised,&black_squares,&white_squares]() {
        //     boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

        //     if (self->ctx.camera.video_alive) {
        //         uint8_t* data0;
        //         auto len0 = cv_helpers::mat2buffer(mask, data0);
        //         self->ctx.imemit(EV_DEBUGCAPTURE, data0, len0, true);

        //         uint8_t* data1;
        //         auto len1=cv_helpers::mat2buffer(denoised, data1);
        //         self->ctx.imemit(EV_DEBUGCAPTURE, data1, len1, true);

        //         uint8_t* data2;
        //         auto len2=cv_helpers::mat2buffer(black_squares, data2);
        //         self->ctx.imemit(EV_DEBUGCAPTURE, data2, len2, true);

        //         uint8_t* data3;
        //         auto len3=cv_helpers::mat2buffer(white_squares, data3);
        //         self->ctx.imemit(EV_DEBUGCAPTURE, data3, len3, true);
        //     }
        // };

        // imupdate();

        int npx = cv::countNonZero(mask);

        if (npx <= 0)
            return false;

        cv::Mat coordinates(cv::Size(2, npx), CV_64FC1);
        int idx = 0;

        for (int i = 0; i < mask.rows; i++)
        {
            for (int j = 0; j < mask.cols; j++)
            {
                if (cv_helpers::get_px(mask, j, i) == cv_helpers::MAX_INTENSITY)
                {
                    coordinates.ptr<double>(idx)[0] = j;
                    coordinates.ptr<double>(idx)[1] = i;
                    idx++;
                }
            }
        }

        Eigen::Vector2d O;
        Eigen::MatrixXd V, D;
        cv_helpers::PCA(coordinates, V, D, O);

        // double noiseratio=denoised.size().width > denoised.size().height?
        // (double)denoised.size().width / denoised.size().height:(double)denoised.size().height / denoised.size().width,
        //     axisratio = (V.row(0).norm() * D(0, 0)) / (V.row(1).norm() * D(1, 1)), scalef = 22;

        // //PREVENT FALSE LASER CLASSIFICATION
        // if (axisratio < scalef * noiseratio)
        //     return false;

        Eigen::Vector2d p = V.row(0);
        p.normalize();
        Eigen::Vector2d A, B;
        double diagonal = sqrt(pow(imlaser.cols, 2) + pow(imlaser.rows, 2));
        double minlen = DBL_MAX;

        for (int sign = -1; sign <= 1; sign += 2, minlen = DBL_MAX)
        {
            for (int i = 0; i < pattern_borders.size(); i++)
            {
                auto pq = pattern_borders[i].a - O;
                auto r = sign * diagonal * p;
                auto s = pattern_borders[i].b - pattern_borders[i].a;
                double a = cross_2D(pq, r);
                double b = cross_2D(pq, s);
                double c = cross_2D(r, s);

                if (c != 0)
                {
                    double t = b / c,
                           u = a / c;

                    if (0 <= t && t <= 1 &&
                        0 <= u && u <= 1)
                    {
                        auto intersection = O + t * r;
                        auto vlen = (intersection - O).norm();

                        if (vlen < minlen)
                        {
                            minlen = vlen;

                            if (sign > 0)
                                A = intersection;
                            else
                                B = intersection;
                        }
                    }
                }
            }
        }

        laser_line = line_segment(A, B);

        return true;
    }

    bool calibrate(const std::vector<Eigen::Vector3d> &camera_plane_points, const std::vector<std::vector<Eigen::Vector3d>> &camera_board_points, const std::vector<double> &boundaries_lower, const std::vector<double> &boundaries_upper, Eigen::Hyperplane<double, 3> &laser_plane, Eigen::Vector3d &direction, Eigen::Vector3d &origin)
    {
        hyperplane_fitting::fit_svd(camera_plane_points, laser_plane);
        int ncorners = camera_board_points.size();
        int npairs = camera_board_points[0].size() - 1;

        typedef struct optimization_data
        {
            const std::vector<std::vector<Eigen::Vector3d>> *points;
        } optimization_data;

        optimization_data *opt_data = new optimization_data;
        opt_data->points = &camera_board_points;
        Eigen::Vector3d A = camera_board_points[0][0], B = camera_board_points[0][1], C = camera_board_points[0][2], D = camera_board_points[0][3];
        std::vector<Eigen::Vector3d> points(4);
        points[0] = A, points[1] = B, points[2] = C, points[3] = D;
        circle c = get_circle(points);
        Eigen::Vector3d u = B - A;
        Eigen::Vector3d v = C - A;
        Eigen::Vector3d q = u.cross(v);
        q.normalize();

        // TODO:make it depend on angle of rotation
        double theta = 2 * M_PI * (1.8 / 360);
        Eigen::Matrix3d R_init = Eigen::AngleAxisd(theta, q).toRotationMatrix().transpose();

        std::vector<double> params(12);
        params[0] = R_init(0, 0), params[1] = R_init(0, 1), params[2] = R_init(0, 2);
        params[3] = R_init(1, 0), params[4] = R_init(1, 1), params[5] = R_init(1, 2);
        params[6] = R_init(2, 0), params[7] = R_init(2, 1), params[8] = R_init(2, 2);
        params[9] = c.center(0), params[10] = c.center(1), params[11] = c.center(2);

        alglib::real_1d_array u_array;
        u_array.setlength(params.size());
        u_array.setcontent(params.size(), params.data());
        const int maxit = 1000;
        alglib::ae_int_t maxits = maxit;
        alglib::ae_int_t info;
        alglib::minlmstate state;
        alglib::minlmreport rep;
        double diffstep = 1e-6;
        double tol = 1e-6;

        alglib::minlmcreatev(npairs * camera_board_points.size() * 3 + 6, u_array, diffstep, state);

        // TODO: no harcoded bounds
        alglib::real_1d_array bndl; // = "[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1000,-1000,-1000]";
        alglib::real_1d_array bndu; // = "[+1,+1,+1,+1,+1,+1,+1,+1,+1,+1000,+1000,+1000]";
        bndl.setlength(boundaries_lower.size());
        bndl.setcontent(boundaries_lower.size(), boundaries_lower.data());
        bndu.setlength(boundaries_upper.size());
        bndu.setcontent(boundaries_upper.size(), boundaries_upper.data());
        alglib::minlmsetbc(state, bndl, bndu);

        // TODO: use jacobian, more accurate solution
        //  alglib::minlmcreatevj(npairs_per_corner*orbit_points.size()*3+6,u_array, state);
        alglib::minlmsetcond(state, tol, maxits);

        auto fn = [](const alglib::real_1d_array &u, alglib::real_1d_array &fi, void *ptr)
        {
            std::cout << u.tostring(6) << std::endl;
            auto od = ((optimization_data *)ptr);
            Eigen::Matrix<double, 3, 3> R;
            R << u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], u[8];
            Eigen::Matrix<double, 3, 1> t;
            t << u[9], u[10], u[11];

            const std::vector<std::vector<Eigen::Vector3d>> &points = *(od->points);
            int ncorners = points.size();
            int npairs = points[0].size() - 1;

            for (int i = 0; i < ncorners; i++)
            {
                for (int j = 0; j < npairs; j++)
                {
                    auto X0 = points[i][j];
                    auto X1 = points[i][j + 1];
                    int offset = (i * npairs + j) * 3;
                    Eigen::Vector3d f = R * (X0 - t) + t - X1;
                    fi[offset] = f(0);
                    fi[offset + 1] = f(1);
                    fi[offset + 2] = f(2);
                }
            }

            int offset = ncorners * npairs * 3;
            Eigen::Matrix3d R_identity = R * R.transpose();
            fi[offset] = R_identity(0, 0) - 1;
            fi[offset + 1] = R_identity(0, 1);
            fi[offset + 2] = R_identity(0, 2);
            fi[offset + 3] = R_identity(1, 1) - 1;
            fi[offset + 4] = R_identity(1, 2);
            fi[offset + 5] = R_identity(2, 2) - 1;
        };

        // alglib::minlmoptimize(state, fn,fn_jac,NULL, opt_data);
        alglib::minlmoptimize(state, fn, NULL, opt_data);
        alglib::minlmresults(state, u_array, rep);
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R(0, 0) = u_array[0], R(0, 1) = u_array[1], R(0, 2) = u_array[2];
        R(1, 0) = u_array[3], R(1, 1) = u_array[4], R(1, 2) = u_array[5];
        R(2, 0) = u_array[6], R(2, 1) = u_array[7], R(2, 2) = u_array[8];
        origin(0) = u_array[9], origin(1) = u_array[10], origin(2) = u_array[11];
        double gamma = acos(0.5 * (R.trace() - 1));

        if (gamma < 1e-9)
        {
            return false;
        }
        else if (M_PI - gamma < 1e-9)
        {
            // TODO: https://www.geometrictools.com/Documentation/RotationRepresentations.pdf
            return false;
        }

        direction = Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        direction.normalize();

        return true;
    }

    // TODO: Think about whether automatic calibration is necessary instead of manual by capturing images by user
    void command_scannercalibstart::execute(std::shared_ptr<command> self)
    {
        ctx.calibrating = true;
        ctx.camera.video_alive = true;
        ctx.camera.camera_alive = true;
        std::shared_ptr<boost::mutex> vcap_mtx(new boost::mutex);
        std::shared_ptr<cv::VideoCapture> cap(new cv::VideoCapture);
        int videoid = camera::get_videoid("USB 2.0 Camera: USB Camera");

        if (videoid == -1)
        {
            std::cerr << "could not detect USB camera" << std::endl;
        }

        cap->open(videoid);
        cap->set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1200);

        if (!cap->isOpened())
        {
            std::cerr << "error opening camera" << std::endl;
        }

        auto K = self->ctx.camera.calib.K;
        auto D = self->ctx.camera.calib.D;

        auto fnvideo = [self, cap, vcap_mtx, K, D]()
        {
            cv::Mat frame, undistorted;
            bool running = true;

            // std::cout<<"KCAM0:"<<self->ctx.camera.calib.K<<std::endl;
            // std::cout<<"DCAM0:"<<self->ctx.camera.calib.D<<std::endl;

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

                    cv::undistort(frame, undistorted, K, D);

                    auto imupdate = [self, &undistorted]()
                    {
                        boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                        if (self->ctx.camera.video_alive)
                        {
                            uint8_t *data;
                            auto len = cv_helpers::mat2buffer(undistorted, data);
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

        auto fncamera = [self, cap, vcap_mtx, K, D]()
        {
            std::vector<cv::Point3d> world_board_points;
            auto pattern_size = self->ctx.sccalib.pattern_size;
            int patternw = pattern_size.width, patternh = pattern_size.height;
            double squarew = self->ctx.sccalib.square_size, squareh = self->ctx.sccalib.square_size;
            int n_calibration_images = self->ctx.sccalib.n_calibration_images;
            int steps_per_calibration_image = self->ctx.sccalib.steps_per_calibration_image;
            double stepper_gear_ratio = self->ctx.sccalib.stepper_gear_ratio;
            double rotation_angle_per_step = ROTATION_RESOLUTION_FULLSTEP / stepper_gear_ratio;
            double rotation_angle_per_calibration_image = steps_per_calibration_image * rotation_angle_per_step;
            double total_rotation_angle = n_calibration_images * rotation_angle_per_calibration_image;
            double start_rotation_angle = 180 - total_rotation_angle / 2;
            start_rotation_angle = start_rotation_angle >= 0 ? start_rotation_angle : 0;
            double start_angle_steps = start_rotation_angle / rotation_angle_per_step;
            int delay_initial_rotation = 20000;
            int delay_calibration_image_rotation = 15000;

            const int max_pose_estimation_attempts = 3;
            const int pose_history_length = 5;

            std::vector<double> boundaries_upper(12), boundaries_lower(12);
            const double origin_bound_lower = -5000, origin_bound_upper = 5000;
            boundaries_lower[0] = -1, boundaries_lower[1] = -1, boundaries_lower[2] = -1;
            boundaries_lower[3] = -1, boundaries_lower[4] = -1, boundaries_lower[5] = -1;
            boundaries_lower[6] = -1, boundaries_lower[7] = -1, boundaries_lower[8] = -1;
            boundaries_lower[9] = origin_bound_lower, boundaries_lower[10] = origin_bound_lower, boundaries_lower[11] = origin_bound_lower;
            boundaries_upper[0] = 1, boundaries_upper[1] = 1, boundaries_upper[2] = 1;
            boundaries_upper[3] = 1, boundaries_upper[4] = 1, boundaries_upper[5] = 1;
            boundaries_upper[6] = 1, boundaries_upper[7] = 1, boundaries_upper[8] = 1;
            boundaries_upper[9] = origin_bound_upper, boundaries_upper[10] = origin_bound_upper, boundaries_upper[11] = origin_bound_upper;

            for (int i = 0; i < patternh; i++)
            {
                for (int j = 0; j < patternw; j++)
                {
                    world_board_points.push_back(cv::Point3d(j * squarew, i * squareh, 0));
                }
            }

            std::vector<std::vector<Eigen::Vector3d>> camera_board_points(world_board_points.size());
            std::vector<std::pair<int, Eigen::Vector3d>> camera_plane_points_by_id;
            bool running = true;

            self->ctx.camera.clear_key_camera();
            self->ctx.camera.clear_messageq_camera();
            int n_current_images = 0;

            while (running)
            {
                try
                {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / FPS_30));
                    int keycode = self->ctx.camera.get_key_camera();
                    nlohmann::json msg, response;
                    bool recieved = self->ctx.camera.recieve_message_camera(msg);

                    if (recieved)
                    {
                        auto data = msg["data"].get<std::string>();
                        // std::cout<<msg.dump()<<std::endl;

                        if (!data.compare("clear"))
                            camera_plane_points_by_id.clear();
                    }

                    // Calibrate
                    if (keycode == KEYCODE_C)
                    {
                        Eigen::Hyperplane<double, 3> laser_plane(Eigen::Vector3d(1, 1, 1), 1);
                        std::vector<Eigen::Vector3d> camera_plane_points(camera_plane_points_by_id.size());

                        for (int i = 0; i < camera_plane_points_by_id.size(); i++)
                        {
                            camera_plane_points[i] = camera_plane_points_by_id[i].second;
                        }

                        Eigen::Vector3d direction, origin;
                        bool success = calibrate(camera_plane_points, camera_board_points, boundaries_lower, boundaries_upper, laser_plane, direction, origin);

                        if (success)
                        {
                            self->ctx.sccalib.laser_plane = laser_plane;
                            Eigen::Matrix<double, 4, 1> n = laser_plane.coeffs();

                            // std::cout<<"svd:"<< "("<<n(3)<<"-XX*("<<n(0)<<")-YY*("<<n(1)<<"))/("<< n(2)<<")"<<std::endl;

                            boost::unique_lock<boost::mutex> lock(self->ctx.mtx_calibrated);
                            self->ctx.calibrated = true;
                            lock.unlock();
                            nlohmann::json j;
                            j["prop"] = PROP_SCANNERCALIBRATED;
                            j["value"] = true;
                            self->ctx.stremit(EV_PROPCHANGED, j.dump(), true);
                            std::vector<double> nvec;
                            nvec.push_back(n(0)), nvec.push_back(n(1)), nvec.push_back(n(2)), nvec.push_back(n(3));
                            j.clear();
                            j["type"] = "plane";
                            j["n"] = nvec;
                            // j["centroid"] = centroid;
                            self->ctx.stremit(EV_SCANNERCALIBDATA, j.dump(), true);

                            std::vector<std::vector<double>> center_points;
                            Eigen::Vector3d a, b;
                            get_rotation_axis(camera_board_points, a, b, center_points);

                            // Eigen::Matrix4d RT;
                            // bool ok=get_rigid_body_transform(camera_board_points,RT,direction,source);
                            // auto RT_rotation_axis=get_rotation_axis_transform(camera_board_points);
                            self->ctx.sccalib.rotation_axis_direction = direction;
                            self->ctx.sccalib.rotation_axis_origin = origin;
                            self->ctx.sccalib.save("scanner_calib.json");

                            std::vector<double> direction_vec, source_vec;
                            direction_vec.push_back(direction(0));
                            direction_vec.push_back(direction(1));
                            direction_vec.push_back(direction(2));
                            source_vec.push_back(origin(0));
                            source_vec.push_back(origin(1));
                            source_vec.push_back(origin(2));

                            std::vector<std::vector<double>> orbit_points;
                            orbit_points.push_back(std::vector<double>());
                            orbit_points.push_back(std::vector<double>());
                            orbit_points.push_back(std::vector<double>());
                            int npoints = camera_board_points[0].size();

                            for (size_t i = 0; i < camera_board_points.size(); i++)
                            {
                                // if((i/3)%2==0) {
                                for (size_t j = 0; j < camera_board_points[0].size(); j++)
                                {
                                    auto v = camera_board_points[i][j];
                                    orbit_points[0].push_back(v(0));
                                    orbit_points[1].push_back(v(1));
                                    orbit_points[2].push_back(v(2));
                                }
                                // }
                            }

                            self->ctx.sccalib.save_points("axis_points.json", direction_vec, source_vec, orbit_points, center_points, npoints);
                        }
                    }

                    if (keycode == KEYCODE_SPACE)
                    {
                        bool err = false;
                        self->ctx.controller.rotate(self->ctx.scconfig.rotation_direction, start_angle_steps, response, delay_initial_rotation, err);

                        while (n_current_images < n_calibration_images)
                        {
                            cv::Mat imnolaser, imnolaseru;
                            err = false;
                            // msg = "laser;" + microcontroller::format("state", 0) + microcontroller::format("delay", 200);
                            self->ctx.controller.set_laser(0, 200, response, 2000, err);

                            // self->ctx.controller.send_message(msg, response, 2000, err);

                            if (err)
                            {
                                self->ctx.stremit(EV_ERROR, "error turning off laser", true);
                                break;
                            }

                            // TODO: could perhaps use kalman filter to improve accuracy instead of this simple but slow solution
                            std::vector<std::vector<cv::Point2f>> image_board_points_history;
                            int attempts = 0;

                            for (int d = 0; d < pose_history_length && attempts <= max_pose_estimation_attempts; d++)
                            {
                                std::vector<cv::Point2f> points;
                                cv::Mat frame, frameu, gray;
                                boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                                cap->read(frame);
                                lock.unlock();

                                if (frame.empty())
                                    continue;

                                cv::undistort(frame, frameu, K, D);
                                cv::cvtColor(frameu, gray, cv::COLOR_BGR2GRAY);
                                bool found = cv::findChessboardCorners(gray, pattern_size, points);

                                if (found)
                                {
                                    attempts = 0;
                                    image_board_points_history.push_back(points);
                                }
                                else
                                    attempts++;
                            }

                            if (attempts > max_pose_estimation_attempts)
                            {
                                self->ctx.stremit(EV_ERROR, "chessboard not found", true);
                                break;
                            }

                            std::vector<cv::Point2f> avg_image_board_points;
                            int npoints = patternw * patternh;

                            for (int i = 0; i < npoints; i++)
                            {
                                float x = 0, y = 0;

                                for (int j = 0; j < image_board_points_history.size(); j++)
                                {
                                    x += image_board_points_history[j][i].x;
                                    y += image_board_points_history[j][i].y;
                                }

                                x /= image_board_points_history.size();
                                y /= image_board_points_history.size();
                                avg_image_board_points.push_back(cv::Point2f(x, y));
                            }

                            cv::Mat rvecs, tvec;
                            bool solved = cv::solvePnP(world_board_points, avg_image_board_points, K, D, rvecs, tvec);

                            if (!solved)
                            {
                                self->ctx.stremit(EV_ERROR, "error estimating pose", true);
                                break;
                            }

                            boost::unique_lock<boost::mutex> lock(*vcap_mtx);
                            cap->read(imnolaser);
                            lock.unlock();

                            if (imnolaser.empty())
                            {
                                self->ctx.stremit(EV_ERROR, "image was empty", true);
                                break;
                            }

                            cv::undistort(imnolaser, imnolaseru, K, D);

                            cv::Mat imlaser, imlaseru;
                            err = false;
                            // msg = "laser;" + microcontroller::format("state", 1) + microcontroller::format("delay", 1000);
                            // self->ctx.controller.send_message(msg, response, 2000, err);
                            self->ctx.controller.set_laser(1, 1000, response, 2000, err);

                            if (err)
                            {
                                self->ctx.stremit(EV_ERROR, "error turning on laser", true);
                                break;
                            }

                            boost::unique_lock<boost::mutex> lock_(*vcap_mtx);
                            cap->read(imlaser);
                            lock_.unlock();

                            if (imlaser.empty())
                            {
                                self->ctx.stremit(EV_ERROR, "image was empty", true);
                                break;
                            }

                            err = false;
                            self->ctx.controller.set_laser(0, 100, response, 2000, err);

                            if (err)
                            {
                                self->ctx.stremit(EV_ERROR, "error turning off laser", true);
                            }

                            cv::undistort(imlaser, imlaseru, K, D);

                            std::vector<Eigen::Vector2d> image_board_corners;

                            for (auto &point : avg_image_board_points)
                            {
                                image_board_corners.push_back(Eigen::Vector2d(point.x, point.y));
                            }

                            Eigen::Vector2d a = image_board_corners[0],
                                            b = image_board_corners[patternw - 1],
                                            c = image_board_corners[(patternh - 1) * patternw],
                                            d = image_board_corners[npoints - 1];
                            std::vector<Eigen::Vector2d> corners;
                            corners.push_back(a), corners.push_back(b),
                                corners.push_back(d), corners.push_back(c);
                            polygon quad(corners);
                            polygon poly(image_board_corners);
                            rectangle frame = quad.frame();
                            polygon translated_poly = poly.translate(-frame.UL); // May be unnecessary check later
                            polygon translated_quad = quad.translate(-frame.UL);
                            cv::Mat imlaser_cropped, cut_mask, imlaser_cut,
                                imnolaser_cropped, imnolaser_cut, diff;
                            cv_helpers::crop(imlaseru, imlaser_cropped, frame.UL, frame.BR);
                            cv_helpers::cut(cut_mask, imlaser_cropped.size(), translated_quad.sides);
                            imlaser_cropped.copyTo(imlaser_cut, cut_mask);
                            cv_helpers::crop(imnolaseru, imnolaser_cropped, frame.UL, frame.BR);
                            imnolaser_cropped.copyTo(imnolaser_cut, cut_mask);
                            line_segment laser_line;
                            bool found_laser = find_laser_line(self, imlaser_cut, imnolaser_cut, translated_poly.vertices, pattern_size, laser_line);

                            if (found_laser)
                            {
                                n_current_images++;
                                auto id = std::to_string(n_current_images);
                                line_segment translated_laser_line = laser_line.translate(frame.UL);
                                cv::line(imlaser, cv::Point(translated_laser_line.a(0), translated_laser_line.a(1)),
                                         cv::Point(translated_laser_line.b(0), translated_laser_line.b(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

                                cv::drawFrameAxes(imlaser, K, D, rvecs, tvec, 20);
                                // uint8_t* data0;
                                uint8_t *data;
                                // auto len0 = cv_helpers::mat2buffer(imlaser_cut, data0);
                                auto len = cv_helpers::mat2buffer(imlaser, data);
                                // self->ctx.imemit(EV_DEBUGCAPTURE, data0, len0, true);
                                self->ctx.imemit(EV_DEBUGCAPTURE, data, len, true);

                                cv::Rodrigues(rvecs, rvecs);
                                Eigen::Matrix<double, 3, 4> T;
                                T << *rvecs.ptr<double>(0), *(rvecs.ptr<double>(0) + 1), *(rvecs.ptr<double>(0) + 2), *tvec.ptr<double>(0),
                                    *rvecs.ptr<double>(1), *(rvecs.ptr<double>(1) + 1), *(rvecs.ptr<double>(1) + 2), *tvec.ptr<double>(1),
                                    *rvecs.ptr<double>(2), *(rvecs.ptr<double>(2) + 1), *(rvecs.ptr<double>(2) + 2), *tvec.ptr<double>(2);
                                Eigen::Matrix3d H;
                                H.col(0) = T.col(0), H.col(1) = T.col(1), H.col(2) = T.col(3);
                                const int step_marker = 3;

                                for (int i = 0; i < patternh; i++)
                                {
                                    int L_idx = i * patternw, R_idx = L_idx + patternw - 1;
                                    Eigen::Vector2d L(avg_image_board_points[L_idx].x, avg_image_board_points[L_idx].y),
                                        R(avg_image_board_points[R_idx].x, avg_image_board_points[R_idx].y),
                                        intersection;
                                    bool intersects = math_helpers::intersection_line_segment(L, R,
                                                                                              translated_laser_line.a, translated_laser_line.b, intersection);

                                    if (intersects)
                                    {
                                        cv::circle(imlaser, cv::Point(intersection(0), intersection(1)), 5, cv::Scalar(255, 0, 0));

                                        int y = i * squareh, a_idx = i * patternw, b_idx = a_idx + step_marker,
                                            c_idx = b_idx + step_marker;
                                        Eigen::Vector2d A(0, y),
                                            B(step_marker * squarew, y),
                                            C(2 * step_marker * squarew, y),
                                            a(avg_image_board_points[a_idx].x, avg_image_board_points[a_idx].y),
                                            b(avg_image_board_points[b_idx].x, avg_image_board_points[b_idx].y),
                                            c(avg_image_board_points[c_idx].x, avg_image_board_points[c_idx].y);

                                        double Q = math_helpers::cross_ratio(A(0), B(0), C(0),
                                                                             0, (b - a).norm(), (c - a).norm(), (intersection - a).norm());
                                        Eigen::Vector3d camera_point = H * Eigen::Vector3d(Q, y, 1);

                                        nlohmann::json j;
                                        std::vector<double> xyz(camera_point.data(), camera_point.data() + camera_point.rows() * camera_point.cols());
                                        j["type"] = "point";
                                        j["xyz"] = xyz;
                                        j["id"] = id;
                                        self->ctx.stremit(EV_SCANNERCALIBDATA, j.dump(), true);
                                        camera_plane_points_by_id.push_back(std::pair<int, Eigen::Vector3d>(n_current_images, camera_point));
                                    }
                                }

                                auto imupdate = [self, &imlaser, id]()
                                {
                                    boost::unique_lock<boost::mutex> lock(self->ctx.camera.mtx_video_alive);

                                    if (self->ctx.camera.video_alive)
                                    {
                                        cv::Mat thumbnail;
                                        cv::resize(imlaser, thumbnail, cv::Size(213, 120));
                                        uint8_t *data0;
                                        uint8_t *data1;
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

                                for (int i = 0; i < camera_board_points.size(); i++)
                                {
                                    Eigen::Vector4d hmg_world_board_point(world_board_points[i].x, world_board_points[i].y, world_board_points[i].z, 1);
                                    Eigen::Vector3d camera_board_point = T * hmg_world_board_point;
                                    camera_board_points[i].push_back(camera_board_point);
                                }

                                self->ctx.controller.rotate(self->ctx.scconfig.rotation_direction, steps_per_calibration_image, response, delay_calibration_image_rotation, err);
                            }
                        }
                    }
                }
                catch (boost::thread_interrupted &)
                {
                    running = false;
                    self->ctx.commandq.enqueue(std::shared_ptr<command>(new command_scannercalibstop(self->ctx, COMM_SCANNERCALIBSTOP)));
                }
            }
        };

        nlohmann::json j;
        j["prop"] = PROP_CALIBRATINGSCANNER;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        j["prop"] = PROP_VIDEOALIVE;
        j["value"] = true;
        ctx.stremit(EV_PROPCHANGED, j.dump(), true);
        ctx.stremit(EV_SCANNERCALIBSTART, "", true);
        ctx.camera.thread_camera = boost::thread{fncamera};
        ctx.camera.thread_video = boost::thread{fnvideo};
    }
}