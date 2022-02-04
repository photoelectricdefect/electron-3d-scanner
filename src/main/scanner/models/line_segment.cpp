#include <models/line_segment.hpp>

	line_segment::line_segment() {};

	line_segment::line_segment(const Eigen::Vector2d& a_, const Eigen::Vector2d& b_) : a(a_), b(b_) { }

	line_segment line_segment::translate(const Eigen::Vector2d& r0) {
		return line_segment(a + r0, b + r0);
	}
