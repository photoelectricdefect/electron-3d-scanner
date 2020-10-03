#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <models/line_segment.hpp>

namespace math_helpers {

// double eucl2D(const cv::Mat& a, const cv::Mat& b);
double eucl2D(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
int cross2D(const Eigen::Vector2d& u, const Eigen::Vector2d& v);
//TODO: improve by accounting for parallel lines
bool intersection_line_segment(const Eigen::Vector2d& a1, const Eigen::Vector2d& b1, const Eigen::Vector2d& a2, const Eigen::Vector2d& b2, Eigen::Vector2d& intersection);
//TODO: improve by accounting for parallel lines
//bool intersects_line_segment(const Eigen::Vector2f& a1, const Eigen::Vector2f& b1, const Eigen::Vector2f& a2, const Eigen::Vector2d& b2);
bool intersection_line(const Eigen::Vector2d& a1, const Eigen::Vector2d& b1, const Eigen::Vector2d& a2, const Eigen::Vector2d& b2, Eigen::Vector2d& intersection);
Eigen::Vector3d proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b, double b_squared_norm);
Eigen::Vector3d proj_a2b3D(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
double cross_ratio(double A, double B, double C, double a, double b, double c, double q);
bool inside_polygon(const Eigen::Vector2f& a, const std::vector<line_segment>& sides);

}