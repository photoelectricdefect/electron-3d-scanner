#include <models/line_segment.hpp>

	line_segment::line_segment() {};

	line_segment::line_segment(const Eigen::Vector2f& _a, const Eigen::Vector2f& _b)
	{
		a = _a;
		b = _b;
	}

	line_segment line_segment::translate(const Eigen::Vector2f& r0) {
		return line_segment(a + r0, b + r0);
	}
