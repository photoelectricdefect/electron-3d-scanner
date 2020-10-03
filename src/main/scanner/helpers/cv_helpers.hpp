#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <models/line_segment.hpp>
#include <string>
#include <vector>

namespace cv_helpers {

const int MIN_INTENSITY = 0;
const int MAX_INTENSITY = 255;
const cv::Vec3b MIN_INTENSITY_3C(0, 0, 0) ;
const cv::Vec3b MAX_INTENSITY_3C(255, 255, 255);

void set_px(const cv::Mat& img, int x, int y, uint8_t intensity);
void set_px(const cv::Mat& img, int x, int y, cv::Vec3b intensity);
uint8_t get_px(const cv::Mat& img, int x, int y);
cv::Vec3b get_px3C(const cv::Mat& img, int x, int y);
void sharpen(const cv::Mat& img, const cv::Mat& sharpened, float alpha, int threshold);
bool inside_polygon(const Eigen::Vector2d& a, const std::vector<line_segment>& sides);
void cut(cv::Mat& mask, cv::Size img_size, const std::vector<line_segment>& borders);
void crop(const cv::Mat& img,  cv::Mat& cropped, const Eigen::Vector2d& q,  const Eigen::Vector2d& s);
// static float masked_threshold(cv::Mat& img, cv::Mat mask, int type, int threshold);
std::string mat2base64str(cv::Mat& img);
void PCA(cv::Mat& data, Eigen::MatrixXd& V, Eigen::MatrixXd& D, Eigen::Vector2d& O);

}