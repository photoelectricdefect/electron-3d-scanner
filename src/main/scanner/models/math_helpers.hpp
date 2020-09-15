#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <models/line_segment.hpp>

namespace math_helpers {

static float eucl2D(const cv::Mat& a, const cv::Mat& b);
static float eucl2D(const Eigen::Vector2f& pt1, const Eigen::Vector2f& pt2);
static int cross2D(const Eigen::Vector2f& u, const Eigen::Vector2f& v);
//TODO: improve by adding parallel intesections
static bool intersection_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2, Eigen::Vector2f& intersection);
//TODO: improve by adding parallel intesections
static bool intersects_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2);
static bool intersection_line(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2f& b2, Eigen::Vector2f& intersection);
static Eigen::Vector3f proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float b_squared_norm);
static Eigen::Vector3f proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
static float cross_ratio(float A, float B, float C, float a, float b, float c, float q);
static bool inside_polygon(const Eigen::Vector2f& a, const std::vector<line_segment>& sides);

}