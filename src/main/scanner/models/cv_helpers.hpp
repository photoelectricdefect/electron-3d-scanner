#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <models/line_segment.hpp>

namespace cv_helpers {

const int MIN_INTENSITY = 0;
const int MAX_INTENSITY = 255;
const cv::Vec3b MIN_INTENSITY_3C(0, 0, 0) ;
const cv::Vec3b MAX_INTENSITY_3C(255, 255, 255);

static void set_px(const cv::Mat& img, int x, int y, uint8_t intensity);
static void set_px(const cv::Mat& img, int x, int y, cv::Vec3b intensity);
static uint8_t get_px(const cv::Mat& img, int x, int y);
static cv::Vec3b get_px_3C(const cv::Mat& img, int x, int y);
static void sharpen(const cv::Mat& img, const cv::Mat& sharpened, float alpha, int threshold);
static bool inside_polygon(const Eigen::Vector2f& a, const std::vector<line_segment>& sides);
static void cut(cv::Mat& mask, const cv::Size& img_size, const std::vector<line_segment>& borders);
static void crop(const cv::Mat& img,  cv::Mat& cropped, const Eigen::Vector2f& q,  const Eigen::Vector2f& s);
static float masked_threshold(cv::Mat& img, cv::Mat mask, int type, int threshold);
static std::string mat2base64str(const cv::Mat& img);
static void PCA(cv::Mat& data, Eigen::MatrixXf& V, Eigen::MatrixXf& D, Eigen::Vector2f& O);

}