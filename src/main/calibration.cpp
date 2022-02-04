#include <nlohmann/json.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/photo.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include <Eigen/Dense>
#include <line_segment.h>
#include <rectangle.h>
#include <polygon.h>
#include <math_helpers.h>
#include <cv_helpers.h>
#include <line_classifier.h>
#include <settings.h>
	// 
#include <TMath.h>
// 
#include <TGraph2D.h>
// 
#include <TRandom.h>
// 
#include <TStyle.h>
// 
#include <TCanvas.h>
// 
#include <TF2.h>
// 
#include <TH1.h>
// 
#include "TApplication.h"
#include <plot.h>

void printvec_f(std::vector<float> vec)
{
	std::cout << "(";

	for (int i = 0; i < vec.size(); i++)
	{
		std::cout << vec[i];

		if (i < vec.size() - 1) std::cout << ",";
	}

	std::cout << ")\n";
}

void printvec_2f(std::vector<std::vector < float>> vec)
{
	for (int i = 0; i < vec.size(); i++)
	{
		std::cout << "(";

		for (int j = 0; j < vec[i].size(); j++)
		{
			std::cout << vec[i][j];

			if (j < vec[i].size() - 1) std::cout << ",";
		}

		std::cout << ")\n";
	}
}

void mat2vec_f(cv::Mat mat, std::vector<float> &v)
{
	v = std::vector<float> (mat.ptr<float> (), mat.ptr<float> () + mat.rows);
}

void mat2vec_2f(cv::Mat mat, std::vector<std::vector < float>> &v)
{
	for (int i = 0; i < mat.rows; i++)
	{
		const float *ptr = mat.ptr<float> (i);
		v.push_back(std::vector<float> (ptr, ptr + mat.cols));
	}
}


// void print_vec3f(Eigen::Vector3f v) {
//     Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
//     std::cout << v.format(clean_fmt) << std::endl;
// }

void fit_plane(std::vector<Eigen::Vector3f > laser_pts, Eigen::Hyperplane<float, 3> &plane)
{
	Eigen::Vector3f c0;

	for (int i = 0; i < laser_pts.size(); i++)
	{
		c0 += laser_pts[i];
	}

	c0 /= laser_pts.size();
	Eigen::MatrixXf A(laser_pts.size(), 3);

	for (int i = 0; i < laser_pts.size(); i++)
	{
		A << laser_pts[i] - c0;
	}

	Eigen::JacobiSVD<Eigen::MatrixXf > svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.computeV();
	Eigen::Vector3f n = svd.matrixV().col(A.rows() - 1);
	n.normalize();
	plane = Eigen::Hyperplane<float, 3> (n, c0);
}

void intersection_line_plane(Eigen::ParametrizedLine<float, 3> line, Eigen::Hyperplane<float, 3> plane, Eigen::Vector3f &intersection)
{
	intersection = line.intersectionPoint(plane);
}

void intersection_line_plane(Eigen::Vector3f n, Eigen::Vector3f T, Eigen::Vector3f q, Eigen::Vector3f q0, Eigen::Vector3f &intersection)
{
	n.normalize(), q.normalize();
	Eigen::Hyperplane<float, 3> plane(n, T);
	Eigen::ParametrizedLine<float, 3> line(q0, q);
	intersection_line_plane(line, plane, intersection);
}

void calibrate_camera(std::vector<std::vector<cv::Point3f>> world_pts, cv::Mat &K, cv::Mat &D, int n_caps, cv::Size board_size, cv::Size square_size)
{
	cv::VideoCapture cap;
	cap.open(0);

	if (!cap.isOpened())
	{
		std::cerr << "error opening camera" << std::endl;
		return;
	}

	std::cout << "space = capture image" << std::endl;

	cv::Mat frame, gray;
	std::vector<std::vector<cv::Point2f>> img_pts;
	bool running = true;
	int fps = 60;
	int caps = 0;
	int esc = 27;
	int space = 32;

	while (running && caps < n_caps)
	{
		char c = cv::waitKey((int)(1000 / fps));
		cap.read(frame);

		if (frame.empty())
		{
			std::cerr << "Empty frame grabbed" << std::endl;
			continue;
		}

		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		std::vector<cv::Point2f > pts;
		bool found = cv::findChessboardCorners(gray, board_size, pts);

		if (found)
		{
			cv::cornerSubPix(gray, pts, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

			if (c == space)
			{
				img_pts.push_back(pts);
				caps++;
			}

			cv::drawChessboardCorners(frame, board_size, cv::Mat(pts), found);
		}

		cv::imshow("frame", frame);
	}

	cv::destroyWindow("frame");
	cap.release();
	cv::Mat rvecs, tvec;
	cv::calibrateCamera(world_pts, img_pts, frame.size(), K, D, rvecs, tvec);
}

// void calibrate_projector(std::vector<cv::Point3f > world_pts, cv::Mat K, cv::Mat distortion_coefficients, cv::Size board_size, cv::Size square_size, Eigen::Hyperplane<float, 3> &plane)
// {
// 	if (board_size.height % 2 != 0)
// 	{
// 		std::cerr << "invalid board size" << std::endl;
// 		return;
// 	}

// 	cv::VideoCapture cap;
// 	cap.open(0);

// 	if (!cap.isOpened())
// 	{
// 		std::cerr << "error opening camera" << std::endl;
// 		return;
// 	}

// 	std::cout << "q = capture no laser image, w = capture laser image, e = find laser line" << std::endl;

// 	Eigen::Matrix3f _K;
// 	_K << *(K.ptr<float> (0)), *(K.ptr<float> (0) + 1), *(K.ptr<float> (0) + 2),
// 		*(K.ptr<float> (1)), *(K.ptr<float> (1) + 1), *(K.ptr<float> (1) + 2),
// 		*(K.ptr<float> (2)), *(K.ptr<float> (2) + 1), *(K.ptr<float> (2) + 2);
// 	cv::Mat frame, gray, undist, laser, no_laser, rvecs, tvec;
// 	std::vector<cv::Point2f > img_pts;
// 	std::vector<Eigen::Vector3f > plane_pts;
// 	int fps = 60;
// 	bool running = true;
// 	int q = 113, w = 119, e = 101;
// 	bool laser_capped = false, no_laser_capped = false, found = false, solved = false, classified = false;

// 	while (running)
// 	{
// 		int ch = cv::waitKey(1);
// 		cap.read(frame);

// 		if (frame.empty())
// 		{
// 			std::cerr << "Empty frame grabbed" << std::endl;
// 			continue;
// 		}

// 		cv::undistort(frame, undist, K, distortion_coefficients);

// 		if (ch == q)
// 		{
// 			no_laser = undist.clone();
// 			no_laser_capped = true;
// 		}

// 		cv::cvtColor(undist, gray, cv::COLOR_BGR2GRAY);
// 		found = cv::findChessboardCorners(gray, board_size, img_pts);
// 		line_classifier classifier;

// 		if (found)
// 		{
// 			cv::drawChessboardCorners(frame, board_size, cv::Mat(img_pts), found);

// 			if (no_laser_capped)
// 			{
// 				solved = cv::solvePnP(world_pts, img_pts, K, distortion_coefficients, rvecs, tvec);

// 				if (solved)
// 				{
// 					int idx_b = board_size.width - 1,
// 						idx_c = (board_size.height - 1) * board_size.width,
// 						idx_d = board_size.width * board_size.height - 1;
// 					std::vector<line_segment> borders;
// 					Eigen::Vector2f a(img_pts[0].x, img_pts[0].y),
// 						b(img_pts[idx_b].x, img_pts[idx_b].y),
// 						c(img_pts[idx_c].x, img_pts[idx_c].y),
// 						d(img_pts[idx_d].x, img_pts[idx_d].y);
// 					int min_x = INT_MAX , max_x = -INT_MAX, min_y = INT_MAX, max_y = -INT_MAX;
// 					std::vector<Eigen::Vector2f> corners;
// 					corners.push_back(a);
// 					corners.push_back(b);
// 					corners.push_back(c);
// 					corners.push_back(d);

// 					for(int i = 0; i < corners.size(); i++) {
// 						if(corners[i](0) < min_x) {
// 							min_x = (int)corners[i](0);
// 						}
// 						if(corners[i](0) > max_x) {
// 							max_x = (int)corners[i](0);
// 						}
// 						if(corners[i](1) < min_y) {
// 							min_y = (int)corners[i](1);
// 						}
// 						if(corners[i](1) > max_y) {
// 							max_y = (int)corners[i](1);
// 						}
// 					}		

// 					Eigen::Vector2i UL_corner(min_x, min_y), BR_corner(max_x, max_y);
// 					Eigen::Vector2f translated_a = a - UL_corner.cast<float>();
// 					Eigen::Vector2f translated_b = b - UL_corner.cast<float>();
// 					Eigen::Vector2f translated_c = c - UL_corner.cast<float>();
// 					Eigen::Vector2f translated_d = d - UL_corner.cast<float>();
// 					borders.push_back(line_segment(translated_a, translated_b));
// 					borders.push_back(line_segment(translated_a, translated_c));
// 					borders.push_back(line_segment(translated_d, translated_b));
// 					borders.push_back(line_segment(translated_d, translated_c));
// 					cv::Mat cropped_laser, cut_laser, cropped_no_laser, cut_no_laser, diff;
// 					crop(undist, cropped_laser, UL_corner, BR_corner);
// 					cut(cropped_laser, cut_laser, borders);
// 					crop(no_laser, cropped_no_laser, UL_corner, BR_corner);
// 					cut(cropped_no_laser, cut_no_laser, borders);
// 					line_classifier classifier;
// 					classified = classifier.classify_chessboard(cut_laser, cut_no_laser, borders);
//           			draw_lines(cut_laser, classifier.segments, cv::Scalar(0, 0, 255), 2);  					
// 					// imshow("cropped no laser", cropped_no_laser);

//           			if (classified)
// 					{
// 						cv::line(cut_laser, cv::Point(classifier.laser_segment.a(0), classifier.laser_segment.a(1)),
// 							cv::Point(classifier.laser_segment.b(0), classifier.laser_segment.b(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
// 						imshow("cut laser", cut_laser);
// 					}
// 				}
// 			}
// 		}

// 		if (ch == w && classified)
// 		{
// 			laser = undist.clone();
// 			laser_capped = true;
// 		}

// 		if (ch == e && laser_capped && no_laser_capped)
// 		{
// 			cv::Rodrigues(rvecs, rvecs);
// 			Eigen::MatrixXf RT(3, 4);
// 			RT << *rvecs.ptr<float> (0), *(rvecs.ptr<float> (0) + 1), *(rvecs.ptr<float> (0) + 2), *tvec.ptr<float> (0),
// 				*rvecs.ptr<float> (1), *(rvecs.ptr<float> (1) + 1), *(rvecs.ptr<float> (1) + 2), *tvec.ptr<float> (1),
// 				*rvecs.ptr<float> (2), *(rvecs.ptr<float> (2) + 1), *(rvecs.ptr<float> (2) + 2), *tvec.ptr<float> (2);

// 			int step = board_size.height / 2;

// 			for (int i = 0; i < board_size.width; i++)
// 			{
// 				int B_idx = (board_size.height - 1) *board_size.width + i;
// 				Eigen::Vector2f T(img_pts[i].x, img_pts[i].y),
// 					B(img_pts[B_idx].x, img_pts[B_idx].y),
// 					intersection;
// 				bool intersects = intersection_line_segment(T, B,
// 					classifier.laser_segment.a, classifier.laser_segment.b, intersection);

// 				if (intersects)
// 				{
// 					int b_idx = i + (step - 1) *board_size.width,
// 						c_idx = i + (2 *step - 1) *board_size.width;
// 					Eigen::Vector2f A(i *square_size.width, 0),
// 						B(i *square_size.width, (step - 1) *square_size.height),
// 						C(i *square_size.width, (2 *step - 1) *square_size.height),
// 						a(img_pts[i].x, img_pts[i].y),
// 						b(img_pts[b_idx].x, img_pts[b_idx].y),
// 						c(img_pts[c_idx].x, img_pts[c_idx].y);
// 					float Q = cross_ratio(A, B, C, a, b, c, intersection);
// 					Eigen::Vector3f world_pt(i *square_size.width, Q, 1);
// 					Eigen::Matrix3f H;
// 					H << RT.col(0), RT.col(1), RT.col(3);
// 					Eigen::Vector3f camera_pt = H * world_pt;
// 					plane_pts.push_back(camera_pt);
// 				}
// 			}

// 			imshow("laser", undist);
// 			laser_capped = false, no_laser_capped = false;
// 		}

// 		found = false, solved = false, classified = false;
// 		imshow("frame", undist);
// 	}

// 	fit_plane(plane_pts, plane);

// 	// std::cout << "n: " << std::endl;
// 	// printvec_f(n);
// 	//	// std::cout << std::endl;
// 	// std::cout << "d = " << d << std::endl;
// 	//TODO: use plotting lib 

// 	//FOR TESTING
// 	//----------
// 	// cv::Size img_size = frame.size();
// 	// int diag = round(sqrt(pow(img_size.height, 2) + pow(img_size.width, 2)));

// 	// for(auto& line : hough_lines)
// 	// {
// 	//     float r = line[0];
// 	//     float sin_theta = sin(line[1]);
// 	//     float cos_theta = cos(line[1]);
// 	//     int x0 = round(r *cos_theta);
// 	//     int y0 = round(r *sin_theta);
// 	//     cv::Point pt1, pt2;
// 	//     pt1.x = cvRound(x0 + diag *sin_theta);
// 	//     pt1.y = cvRound(y0 - diag *cos_theta);
// 	//     pt2.x = cvRound(x0 - diag *sin_theta);
// 	//     pt2.y = cvRound(y0 + diag *cos_theta);
// 	//     cv::line(frame, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
// 	// }

// 	//------------------------------

// }

void estimate_coordinates(const Eigen::Matrix3f& K, const Eigen::Hyperplane<float, 3>& plane, const std::vector<Eigen::Vector3f >& hmg_img_pts, std::vector< Eigen::Vector3f > &estimated_pts)
{
	Eigen::Vector3f O(0, 0, 0);

	for (auto &pt: hmg_img_pts)
	{
		Eigen::Vector3f q = K.inverse() * pt, intersection;
		q.normalize();
		intersection_line_plane(Eigen::ParametrizedLine<float, 3> (O, q), plane, intersection);
		estimated_pts.push_back(intersection);
	}
}

void test_scanner(std::vector<cv::Point3f > world_pts, cv::Mat K, cv::Mat distortion_coefficients, cv::Size board_size, cv::Size square_size)
{
	assert(board_size.height >= 3 && board_size.width >= 3);

	cv::VideoCapture cap;
	cap.open(0);

	if (!cap.isOpened())
	{
	    std::cout << std::endl;		
		std::cerr << "error opening camera" << std::endl;
		return;
	}

	std::cout << std::endl;
	std::cout << "q = capture no laser image, w = capture laser image, e = find laser line" << std::endl;
	Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> _K(K.ptr<double>(), K.rows, K.cols);
	cv::Mat frame, gray, undist, laser, no_laser, rvecs, tvec;
	std::vector<cv::Point2f > img_pts;
	std::vector<Eigen::Vector3f > plane_pts;
	int fps = 60;
	bool running = true;
	int q = 113, w = 119, e = 101;
	bool laser_capped = false, no_laser_capped = false, found = false, solved = false, classified = false;

	while (running)
	{
		int ch = cv::waitKey(1);

		if(ch == w) {
			running = false;
			continue;
		}

		cap.read(frame);

		if (frame.empty())
		{
			std::cerr << "Empty frame grabbed" << std::endl;
			continue;
		}

		cv::undistort(frame, undist, K, distortion_coefficients);

		if (ch == q)
		{
			no_laser = undist.clone();
			no_laser_capped = true;
		}

		cv::cvtColor(undist, gray, cv::COLOR_BGR2GRAY);
		found = cv::findChessboardCorners(gray, board_size, img_pts);
		line_classifier classifier;

		if (found)
		{
			cv::drawChessboardCorners(frame, board_size, cv::Mat(img_pts), found);

			if (no_laser_capped)
			{
				solved = cv::solvePnP(world_pts, img_pts, K, distortion_coefficients, rvecs, tvec);

				if (solved)
				{
					std::vector<Eigen::Vector2f> corners;

					for(auto& pt : img_pts) {
						corners.push_back(Eigen::Vector2f(pt.x, pt.y));
					}

					Eigen::Vector2f a = corners[0],
						b = corners[board_size.width - 1],
						c = corners[(board_size.height - 1) * board_size.width],
						d = corners[board_size.width * board_size.height - 1];
					std::vector<Eigen::Vector2f> outer_corners;
					outer_corners.push_back(a), outer_corners.push_back(b),
					outer_corners.push_back(d), outer_corners.push_back(c);
					polygon quad(outer_corners);
					polygon poly(corners);
					rectangle frame = poly.frame();
					polygon translated_poly = poly.translate(-frame.UL);
					polygon translated_quad = quad.translate(-frame.UL);
					cv::Mat laser_cropped, laser_mask, laser_cut, 
					no_laser_cropped, no_laser_mask, no_laser_cut, diff;
					crop(undist, laser_cropped, frame.UL, frame.BR);
					cut(laser_mask, laser_cropped.size(), translated_quad.sides);
					laser_cropped.copyTo(laser_cut, laser_mask);
					crop(no_laser, no_laser_cropped, frame.UL, frame.BR);
					cut(no_laser_mask, no_laser_cropped.size(), translated_quad.sides);
					no_laser_cropped.copyTo(no_laser_cut, no_laser_mask);
					line_classifier classifier;
					classified = classifier.classify_chessboard(laser_cut, no_laser_cut, translated_poly.vertices, board_size);

          			if (classified)
					{
						line_segment translated_laser = classifier.laser_segment.translate(frame.UL);
						cv::line(undist, cv::Point(translated_laser.a(0), translated_laser.a(1)),
							cv::Point(translated_laser.b(0), translated_laser.b(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

						cv::Rodrigues(rvecs, rvecs);
						Eigen::Matrix<double, 3, 4> RT;
						RT << *rvecs.ptr<double> (0), *(rvecs.ptr<double> (0) + 1), *(rvecs.ptr<double> (0) + 2), *tvec.ptr<double> (0),
						*rvecs.ptr<double> (1), *(rvecs.ptr<double> (1) + 1), *(rvecs.ptr<double> (1) + 2), *tvec.ptr<double> (1),
						*rvecs.ptr<double> (2), *(rvecs.ptr<double> (2) + 1), *(rvecs.ptr<double> (2) + 2), *tvec.ptr<double> (2);
						Eigen::Matrix3d H;
						H.col(0) = RT.col(0), H.col(1) = RT.col(1), H.col(2) = RT.col(3);

						int step_y = board_size.height / 3;

			for (int i = 0; i < board_size.width; i++)
			{
				int B_idx = (board_size.height - 1) * board_size.width + i;
				Eigen::Vector2f T(img_pts[i].x, img_pts[i].y),
					B(img_pts[B_idx].x, img_pts[B_idx].y),
					intersection;
				bool intersects = intersection_line_segment(T, B,
					translated_laser.a, translated_laser.b, intersection);

						cv::line(undist, cv::Point(T(0), T(1)),
							cv::Point(B(0), B(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

				if (intersects)
				{
					cv::circle(undist, cv::Point(intersection(0), intersection(1)), 5, cv::Scalar(255, 0, 0));

					int x = i *square_size.width, b_idx = i + step_y * board_size.width,
						c_idx = i + 2 *step_y * board_size.width;
					Eigen::Vector2f A(x, 0),
						B(x, step_y  *square_size.height),
						C(x, 2 * step_y *square_size.height),
						a(img_pts[i].x, img_pts[i].y),
						b(img_pts[b_idx].x, img_pts[b_idx].y),
						c(img_pts[c_idx].x, img_pts[c_idx].y);

					float Q = cross_ratio(A(1), B(1), C(1), 
					0, (b - a).norm(), (c - a).norm(), (intersection - a).norm());
					Eigen::Vector3d world_pt(x, Q, 1), camera_pt = H * world_pt;
					std::cout << "board coords: " << std::endl << world_pt << std::endl;
					plane_pts.push_back(camera_pt.cast<float>());

					std::cout << "world point vertical: " << std::endl << world_pt << std::endl;
					std::cout << "camera point vertical: " << std::endl << camera_pt << std::endl;

					if(ch == e) plane_pts.push_back(camera_pt.cast<float>());
				}
			}

			int step_x = board_size.width / 3;

			for (int i = 0; i < board_size.height; i++)
			{
				int L_idx = i * board_size.width, R_idx = L_idx + board_size.width - 1;
				Eigen::Vector2f L(img_pts[L_idx].x, img_pts[L_idx].y),
					R(img_pts[R_idx].x, img_pts[R_idx].y),
					intersection;
				bool intersects = intersection_line_segment(L, R,
					translated_laser.a, translated_laser.b, intersection);

						cv::line(undist, cv::Point(L(0), L(1)),
							cv::Point(R(0), R(1)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

				if (intersects)
				{
					cv::circle(undist, cv::Point(intersection(0), intersection(1)), 5, cv::Scalar(255, 0, 0));

					int y = i * square_size.height, a_idx = i * board_size.width, b_idx = a_idx + step_x,
						c_idx = b_idx + step_x;
					Eigen::Vector2f A(0, y),
						B(step_x * square_size.width, y),
						C(2 * step_x * square_size.width, y),
						a(img_pts[a_idx].x, img_pts[a_idx].y),
						b(img_pts[b_idx].x, img_pts[b_idx].y),
						c(img_pts[c_idx].x, img_pts[c_idx].y);

					float Q = cross_ratio(A(0), B(0), C(0), 
					0, (b - a).norm(), (c - a).norm(), (intersection - a).norm());
					Eigen::Vector3d world_pt(Q, y, 1), camera_pt = H * world_pt;
					
					std::cout << "world point horitontal: " << std::endl << world_pt << std::endl;
					std::cout << "camera point horizontal: " << std::endl << camera_pt << std::endl;

					if(ch == e) plane_pts.push_back(camera_pt.cast<float>());
				}
			}
					
					}
				}
			}
		}

		found = solved = classified = false;
		imshow("frame", undist);
		// std::cout << "frame" << std::endl;
	}

	// for(int i = 0; i < plane_pts.size(); i++) {
	// 	std::cout << "pt: " << std::endl << plane_pts[i] << std::endl;
	// }


	//TODO: check why num is nan sometimes
	for(int i = 0; i < plane_pts.size(); i++) {
		if(isnan(plane_pts[i](0)) || isnan(plane_pts[i](1)) || isnan(plane_pts[i](2))) plane_pts.erase(plane_pts.begin() + i);
	}
	
	plot2D(plane_pts);

	//Eigen::Hyperplane<float, 3> plane;
	//fit_plane(plane_pts, plane);
	
	


	// plot_pts(plane_pts);

	// std::vector<Eigen::Vector3f > _img_pts;

	// for(int i = 0; i < img_pts.size(); i++) {
	//     _img_pts.push_back(Eigen::Vector3f(*_img_pts[i].ptr < float>(0), *_img_pts[i].ptr < float>(1), 1));
	// }

	// std::vector<Eigen::Vector3f > _estimated_pts;
	// estimate_coordinates(_K, plane, _img_pts, estimated_pts);

	// for(int i = 0; i < _estimated_pts.size(); i++) {
	//     std::cout << "(" << _estimated_pts[i](0) 
	//                     << _estimated_pts[i](1) 
	//                     << _estimated_pts[i](2) 
	//                     << ")\n" 
	//                     << std::endl;
	// }
}

int main(int argc, char **argv)
{
	std::cout << "Calibrating camera" << std::endl;
 	// cv::namedWindow( "thresh", cv::WINDOW_AUTOSIZE );
	// init_morph_settings();
	cv::Mat K, D;
	std::vector<cv::Mat > rvecs, tvecs;
	cv::Size board_size(9, 6);
	cv::Size square_size(23, 23);
	int n_caps = 20;
	//they have to be sorted in row major order
	std::vector<cv::Point3f > tmp;

	for (int i = 0; i < board_size.height; i++)
	{
		for (int j = 0; j < board_size.width; j++)
		{
			tmp.push_back(cv::Point3f(j *square_size.width, i *square_size.height, 0));
		}
	}

	std::vector<std::vector<cv::Point3f>> world_pts(n_caps, tmp);
	int c, q = 113, w = 119;

	// std::vector<Eigen::Vector3f> plane_pts;
	// int sign = 1;

	// for(int i = 0; i < 500; i++) {
	// 	plane_pts.push_back(sign * Eigen::Vector3f(rand() % 200, rand() % 200, rand() % 200));
	// 	sign *= -1; 
	// }

	// plot2D(plane_pts);

	while(true) {
		c = getchar();

		if(c == q) {
			calibrate_camera(world_pts, K, D, n_caps, board_size, square_size);
			save_calib(K, D);
			break;
		}
		else if(c == w) {
			load_calib(K, D);
			break;
		}
	}
	
	// std::cout << "k: " << std::endl << K << std::endl;

	// calibrate_projector(K, distortion_coefficients, board_size, square_size);
	test_scanner(tmp, K, D, board_size, square_size);

	// cv::VideoCapture capture;
	// capture.open(0);

	// if(!capture.isOpened())
	// {
	//     std::cerr << "Error opening camera" << std::endl;
	//     return -1;
	// }

	// std::vector<cv::PoiPoint3f > worl*img_pts[i].ptr < float>(0), d_pts;mg_pts[i].ptr < float>(0), d_1t

	// for(int i = 0; i < board_size.height; i++)
	// {
	//     for(int j = 0; j < board_size.width; j++)
	//     {
	//         world_pts.push_back(cv::Point3f(j, i, 0));
	//     }
	// }

	// bool running = true;
	// int c;
	// cv::Mat frame;
	// cv::Mat undst;

	// while(running)
	// {
	//     c = cv::waitKey(10);
	//     capture.read(frame);

	//     if(frame.empty())
	//     {
	//         std::cerr << "Empty frame grabbed" << std::endl;
	//         continue;
	//     }

	//     cv::undistort(frame, undst, K, dist_coeffs);
	//     cv::Mat gray;
	//     cv::cvtColor(undst, gray, cv::COLOR_BGR2GRAY);
	//     bool found = cv::findChessboardCorners(gray, board_(undst, board_size, cv)
	//img_pts[i].ptr < float>(0), / mg_pts[i].ptr < float>(0)1     {
	//         cv::cornerSubPix(), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
	//         cv::Mat rv*img_pts[i].ptr < float>(0), ecs,mg_pts[i].ptr < float>(0), 1c tvec;
	//         cv::solvePnP(world tvec);
	//         cv::*img_pts[i].ptr < float>(0), :*img_pts[i].ptr < float>(1)Mat RT;
	//         cv::Rodrigues(rvecs, rvecs);
	//         cv::hconcat(rvecs, tvec, RT);
	//         cv::Mat C = K * RT;
	//         std::vector<cv::Mat > tmp;

	//         for(auto& img pt(2, 1, CV_32FC1);
	//             *p*img_pts[i].ptr < float>(0), t.mg_pts[i].ptr < float>(0)1 ptr < float>(0) = img_pt.x;
	//             *pt.ptr < float>(1) = img_pt.y;
	//             tmp.push_back(pt);
	//         }

	//         calibrate_projector(frame, tmp, K, RT, board_size);

	//        	// cv::Mat h_o, h_p1, h_p2, h_p3;
	//        	// std::vector<double> _h_o, _h_p1, _h_p2, _h_p3;
	//        	// mat2vec_d(C *cv::Mat(cv::Vec4d(0, 0, 0, 1)), _h_o);
	//        	// mat2vec_d(C *cv::Mat(cv::Vec4d(10, 0, 0, 1)), _h_p1);
	//        	// mat2vec_d(C *cv::Mat(cv::Vec4d(0, 5, 0, 1)), _h_p2);
	//        	// mat2vec_d(C *cv::Mat(cv::Vec4d(0, 0, 5, 1)), _h_p3);

	//        	// cv::Point o(cvRound(_h_o[0] / _h_o[2]), cvRound(_h_o[1] / _h_o[2]));
	//        	// cv::Point p1(cvRound(_h_p1[0] / _h_p1[2]), cvRound(_h_p1[1] / _h_p1[2]));
	//        	// cv::Point p2(cvRound(_h_p2[0] / _h_p2[2]), cvRound(_h_p2[1] / _h_p2[2]));
	//        	// cv::Point p3(cvRound(_h_p3[0] / _h_p3[2]), cvRound(_h_p3[1] / _h_p3[2]));

	//        	// cv::line(undst, o, p1, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
	//        	// cv::line(undst, o, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
	//        	// cv::line(undst, o, p3, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
	//     }

	//     cv::imshow("distorted", frame);
	//     cv::imshow("undistorted", undst);
	// }

	return 0;
}