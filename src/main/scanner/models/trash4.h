// int morph_elem = 0;
// int morph_size = 0;
// int morph_op = 0;
// int morph_iter = 0;
// int thresh = 0;
// int const max_operator = 4;
// int const max_elem = 2;
// int const max_kernel_size = 21;
// int const max_iter = 15;
// int const max_thresh = 255;
// char* window_name = "thresh";

// cv::Mat src, dst;

// void print_vec3f(Eigen::Vector3f v) {
//     Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
//     std::cout << v.format(clean_fmt) << std::endl;
// }

// void morph_settings(int, void*) {
// 	// int op = 2 + morph_op;
// 	// cv::Mat struct_el = cv::getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
// 	// cv::morphologyEx(src, dst, op, struct_el, cv::Point(-1,-1), morph_iter);
// 	cv::threshold( src, dst, thresh, 255, cv::THRESH_BINARY);
// 	cv::imshow("thresh", dst);
// }

// void init_morph_settings() {
// 	cv::createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_name, &morph_op, max_operator, morph_settings);
//  /// Create Trackbar to select kernel type
//  	cv::createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_name,
//                  &morph_elem, max_elem,
//                  morph_settings);

//  /// Create Trackbar to choose kernel size
//  	cv::createTrackbar( "Kernel size:\n 2n +1", window_name,
//                  &morph_size, max_kernel_size,
//                  morph_settings );
	
// 	 	cv::createTrackbar( "Iter:\n", window_name,
//                  &morph_iter, max_iter,
//                  morph_settings );
	
// 		 	cv::createTrackbar( "Thresh:\n", window_name,
//                  &thresh, max_thresh,
//                  morph_settings );
// }

// class line_classifier
// {
// 	public:
// 	static const float gamma;
// 	static const int threshold_low;
// 	std::vector<line_segment> segments;
// 	line_segment laser_segment;


// 	line_classifier() {};

// 	void max_mean_ROI(cv::Mat img, cv::Rect& ROI, int win_size, int step) {
// 		ROI = cv::Rect(0, 0, 0, 0);
// 		int max_i = img.size().height - win_size,
// 		max_j = img.size().width - win_size;
// 		float max_mean = -FLT_MAX;

// 		for(int i = 0; i <= max_i; i += step) {
// 			for(int j = 0; j <= max_j; j += step) {
// 				cv::Rect roi(j, i, win_size, win_size);
// 				cv::Scalar mean = cv::mean(img(roi));
// 				cv::pow(mean, 2, mean);
// 				float mean_len = sqrt(cv::sum(mean)(0));

// 				if(mean_len > max_mean) {
// 					max_mean = mean_len;
// 					ROI = roi;
// 				}
// 			}
// 		}
// 	}

// 	void show_axis(cv::Mat& img, const Eigen::MatrixXf& V, const Eigen::MatrixXf& D, const Eigen::Vector2f& O) {
// 		Eigen::Vector2f a(V(0, 0) * D(0, 0), V(0, 1) * D(0, 0)), b(V(1, 0) * D(1, 1), V(1, 1) * D(1, 1));
// 		a.normalize(), b.normalize();
// 		float scale = 50;
// 		a = scale * a, b = scale * b;
// 		cv::Point _O(O(0), O(1)), p = _O + cv::Point(a(0), a(1)), 
// 		q = _O + cv::Point(b(0), b(1));
// 		cv::Scalar colour(0, 255, 255);
//     	cv::line(img, _O, p, colour, 1, cv::LINE_AA);
//     	cv::line(img, _O, q, colour, 1, cv::LINE_AA);
// 	}

// 	bool classify_chessboard(cv::Mat& laser, cv::Mat no_laser, std::vector<Eigen::Vector2f> corners, cv::Size board_size)
// 	{
// 		cv::Mat diff, mask_low, mask_morph, denoised, morphed, struct_el = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));  
// 		cv::Mat base_colors[3];
// 		diff = cv::abs(laser - no_laser);
// 		cv::split(diff, base_colors);
// 		cv::medianBlur(base_colors[2], denoised, 3);
// 		cv::threshold(denoised, mask_low, threshold_low, 255, cv::THRESH_BINARY);
// 		cv::morphologyEx(mask_low, mask_morph, cv::MORPH_OPEN, struct_el);
// 		denoised.copyTo(morphed, mask_morph);
// 		cv::Rect ROI;
// 		max_mean_ROI(morphed, ROI, 40, 15);
// 		cv::Mat mask, mask_roi;
// 		int threshold = cv::threshold(morphed(ROI), mask_roi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
// 		cv::threshold(morphed, mask, threshold, 255, cv::THRESH_BINARY);
// 		int n_px = cv::countNonZero(mask);
// 		std::cout << "sss" << std::endl;

// 		if(n_px > 0) {
// 		cv::Mat coords(cv::Size(2, n_px), CV_64FC1);
// 		int idx = 0;

// 		for(int i = 0; i < mask.rows; i++) {
// 			for(int j = 0; j < mask.cols; j++) {
// 				if(get_px(mask, j, i) == MAX_INTENSITY) {
// 					coords.ptr<double>(idx)[0] = j;
// 					coords.ptr<double>(idx)[1] = i;
// 					idx++;
// 				}
// 			}
// 		}

// 		Eigen::Vector2f O;
// 		Eigen::MatrixXf V, D;
// 		PCA(coords, V, D, O);
// 		float whr, 
// 		axisr = (V.row(0).norm() * D(0, 0)) / (V.row(1).norm() * D(1,1)), scalef = 22;

// 		if(morphed.size().width > morphed.size().height) whr = (float)morphed.size().width / morphed.size().height;
// 		else whr = (float)morphed.size().height / morphed.size().width;

// 		show_axis(diff, V, D, O);
// 		cv::imshow("morphed", morphed);
// 		cv::imshow("mask", mask);
// 		cv::imshow("diff", diff);
// 		std::cout << "axisr: "<< axisr << std::endl;
// 		std::cout << "whr: "<< whr << std::endl;

// 		if(axisr < scalef * whr) return false;

// 		int idxB = board_size.width - 1, idxC = (board_size.height - 1) * board_size.width,  
// 		idxD = board_size.width * board_size.height - 1;
// 		std::vector<line_segment> borders;
// 		borders.push_back(line_segment(corners[0], corners[idxB]));
// 		borders.push_back(line_segment(corners[0], corners[idxC]));
// 		borders.push_back(line_segment(corners[idxD], corners[idxB]));
// 		borders.push_back(line_segment(corners[idxD], corners[idxC]));
// 		Eigen::Vector2f p = V.row(0);
// 		p.normalize();
// 		std::vector<Eigen::Vector2f> intersections(2, Eigen::Vector2f(FLT_MAX, FLT_MAX));

// 		for(int i = 0; i < borders.size(); i++) {
// 			Eigen::Vector2f intersection; 
// 			bool intersects = intersection_line(borders[i].a, borders[i].b, O, O + p, intersection);

// 			if(intersects) {
// 				float d = (intersection - O).norm();

// 				for(int j = 0; j < intersections.size(); j++) {
// 					if(d < (intersections[j] - O).norm()) {
// 						intersections.insert(intersections.begin() + j, intersection);

// 						if(intersections.size() > 2) intersections.pop_back();
					
// 						break;
// 					}
// 				}
// 			}
// 		}

// 		laser_segment = line_segment(intersections[0], intersections[1]);

// 		return true;
// 		}
// 	}

// };

// const float line_classifier::gamma = 0.12;
// const int line_classifier::threshold_low = 15;
