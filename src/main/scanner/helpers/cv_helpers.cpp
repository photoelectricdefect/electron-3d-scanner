#include <base64.h>
#include <opencv2/imgproc.hpp>

namespace cv_helpers {

static void set_px(const cv::Mat& img, int x, int y, uint8_t intensity) {
	img.ptr<uint8_t>(y)[x] = intensity;
}

static void set_px(const cv::Mat& img, int x, int y, cv::Vec3b intensity) {
	img.ptr<cv::Vec3b>(y)[x] = intensity;
}

static uint8_t get_px(const cv::Mat& img, int x, int y) {
	return img.ptr<uint8_t>(y)[x];
}

static cv::Vec3b get_px_3C(const cv::Mat& img, int x, int y) {
	return img.ptr<cv::Vec3b>(y)[x];
}

static void sharpen(const cv::Mat& img, const cv::Mat& sharpened, float alpha, int threshold) {
	cv::GaussianBlur(img, sharpened, cv::Size(5, 5), 0);
	cv::addWeighted(img, alpha, sharpened, 1 - alpha, 0, sharpened);
	cv::Mat low_contrast_mask = cv::abs(img - sharpened) < threshold;
	img.copyTo(sharpened, low_contrast_mask);
}

//FIX
// float std_dev(cv::Mat m) {
// 	cv::Mat m_blurred, m_squared_blurred;
// 	cv::blur(m, m_blurred, cv::Size(3, 3)), cv::blur(m.mul(m), m_squared_blurred, cv::Size(3, 3));
// 	float sigma = cv::sqrt(m_squared_blurred - m_blurred.mul(m_blurred));

// 	return sigma;
// }

static bool inside_polygon(const Eigen::Vector2f& a, const std::vector<line_segment>& sides) {
			int intersections = 0;
			Eigen::Vector2f O(0, a(1));
			bool on_line = false;

			for(int i = 0; i < sides.size(); i++) {
				Eigen::Vector2f pq = O - sides[i].a,
				r = sides[i].b - sides[i].a,
				s = a - O;
				float e = cross_2D(pq, r),
				b = cross_2D(pq, s),
				c = cross_2D(r, s);

				if (c != 0)
				{
					float t = b / c,
					u = e / c;

					if (0 <= t && t <= 1 &&
					0 <= u && u <= 1)
					{
						intersections++;

						if(u == 1)  {
							on_line = true;
							break;
						}
					}
				}
			}	

			if(intersections % 2 > 0 && !on_line) return true;
			return false;
}

static void cut(cv::Mat& mask, const cv::Size& img_size, const std::vector<line_segment>& borders) {
	mask = cv::Mat(img_size, CV_8UC1);

	for(int i = 0; i < img_size.height; i++) {
		for(int j = 0; j < img_size.width; j++) {
			bool inside = inside_polygon(Eigen::Vector2f(j, i), borders);

			if(!inside) set_px(mask, j, i, MIN_INTENSITY);
			else set_px(mask, j, i, MAX_INTENSITY);
		}
	}
}

static void crop(const cv::Mat& img,  cv::Mat& cropped, const Eigen::Vector2f& q,  const Eigen::Vector2f& s) {	
	cv::Rect ROI(q(0), q(1), abs(s(0) - q(0)), abs(s(1) - q(1)));
	cropped = img(ROI);
}

static float masked_threshold(cv::Mat& img, cv::Mat mask, int type, int threshold) {
	int idx = 0,
	n_px = cv::countNonZero(mask);
	cv::Mat tmp(cv::Size(n_px, 1), CV_8UC1);
	
	for(int c = 0; c < mask.size().height; c++) {
		for(int e = 0; e < mask.size().width; e++) {
			if(get_px(mask, e, c) == MAX_INTENSITY) {
				set_px(tmp, idx, 0, get_px(img, e, c));
				idx++;
			} 
		}
	}

	float t = cv::threshold(tmp, tmp, threshold, 255, type);
	idx = 0;

	for(int c = 0; c < mask.size().height; c++) {
		for(int e = 0; e < mask.size().width; e++) {
			if(get_px(mask, e, c) == MAX_INTENSITY) {
				set_px(img, e, c, get_px(tmp, idx, 0));
				idx++;
			} 
		}
	}

	return t;
}

static void PCA(cv::Mat& data, Eigen::MatrixXf& V, Eigen::MatrixXf& D, Eigen::Vector2f& O) {
	cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> v(pca.eigenvectors.ptr<double>(), data.cols, data.cols);
	V = v.cast<float>();
	O = Eigen::Vector2f(pca.mean.ptr<double>(0)[0], pca.mean.ptr<double>(0)[1]);
	D = Eigen::MatrixXf(data.cols, data.cols);

	for(int i = 0; i < data.cols; i++) {		
		D(i, i) = *pca.eigenvalues.ptr<double>(i);
	}
}

static void mat2base64str(const cv::Mat& img) {
    std::vector<uint8_t> buf;
    cv::imencode(".jpg", img, buf);
    auto *enc = reinterpret_cast<unsigned char*>(buf.data());
    return base64_encode(enc, buf.size());
}

//Should keep??
void find_lines(cv::Mat img, std::vector<cv::Vec2f >& lines, float r, float theta, int hough_threshold, int low_treshold, int high_threshold, int kernel_size)
{
	cv::Mat edges;
	cv::Canny(img, edges, low_treshold, high_threshold, kernel_size);
    cv::imshow("edges", edges);
	cv::HoughLines(edges, lines, r, theta, hough_threshold, 0, 0);
}

//Should keep??
void draw_lines(cv::Mat img, std::vector<cv::Vec2f > lines, cv::Scalar clr, int width)
{
	int diag = round(sqrt(pow(img.size().height, 2) + pow(img.size().width, 2)));

	for (auto &line: lines)
	{
		float r = line[0];
		float sin_theta = sin(line[1]);
		float cos_theta = cos(line[1]);
		int x0 = round(r *cos_theta);
		int y0 = round(r *sin_theta);
		cv::Point a, b;
		a.x = cvRound(x0 + diag *sin_theta);
		a.y = cvRound(y0 - diag *cos_theta);
		b.x = cvRound(x0 - diag *sin_theta);
		b.y = cvRound(y0 + diag *cos_theta);
		cv::line(img, a, b, clr, width, cv::LINE_AA);
	}
}

//Should keep??
void triangulation_matting(cv::Mat B1, cv::Mat I1, cv::Mat B2, cv::Mat I2, cv::Mat& alpha) {
	alpha = cv::Mat(I1.size(), CV_8UC1);
	Eigen::Matrix<float, 6, 4> A;
	A << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0;
	Eigen::Matrix<float, 6, 1> b;
	Eigen::Matrix<float, 4, 1> x;

	for(int i = 0; i < B1.rows; i++) {
		for(int j = 0; j < B1.cols; j++) {
			cv::Vec3b I1_px = get_px_3C(I1, j, i);
			cv::Vec3b B1_px = get_px_3C(B1, j, i);
			cv::Vec3b I2_px = get_px_3C(I2, j, i);
			cv::Vec3b B2_px = get_px_3C(B2, j, i);
			b(0) = I1_px(0) - B1_px(0);
			b(1) = I1_px(1) - B1_px(1);
			b(2) = I1_px(2) - B1_px(2);
			b(3) = I2_px(0) - B2_px(0);
			b(4) = I2_px(1) - B2_px(1);
			b(5) = I2_px(2) - B2_px(2);
			A(0, 3) = -B1_px(0);	
			A(1, 3) = -B1_px(1);
			A(2, 3) = -B1_px(2);
			A(3, 3) = -B2_px(0);
			A(4, 3) = -B2_px(1);
			A(5, 3) = -B2_px(2);	
			x = A.colPivHouseholderQr().solve(b);

			// // x(3) = abs(x(3));
			if (x(0) < 0) { x(0) = 0; }
			else if (x(0) > 255) { x(0) = 255; }
						
			if (x(1) < 0) { x(1) = 0; }
			else if (x(1) > 255) { x(1) = 255; }
						
			if (x(2) < 0) { x(2) = 0; }
			else if (x(2) > 255) { x(2) = 255; }
						
			if (x(3) < 0) { x(3) = 0; }
			else if (x(3) > 1) { x(3) = 1; }

			set_px(alpha, j, i, x(3) * MAX_INTENSITY);
		}

	}
}

//Should keep??
void draw_lines(cv::Mat img, std::vector<line_segment> lines, cv::Scalar clr, int width)
{
	for (auto &ln: lines)
	{
		cv::line(img, cv::Point(ln.a(0), ln.a(1)), cv::Point(ln.b(0), ln.b(1)), clr, width, cv::LINE_AA);
	}
}

//Should keep??
void SLIC(cv::Mat img, cv::Mat& dst, cv::ximgproc::SLICType type, int size, int connectivity) {
	cv::Ptr<cv::ximgproc::SuperpixelSLIC> SLIC = cv::ximgproc::createSuperpixelSLIC(img, type, size, connectivity);
	SLIC->iterate();
	SLIC->enforceLabelConnectivity(50);
	cv::Mat mask;
	SLIC->getLabelContourMask(mask, true);
	dst.setTo(cv::Scalar(0, 255, 0), mask);
}
}