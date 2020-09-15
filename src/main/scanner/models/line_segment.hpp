#ifndef LINESEG_H_
#define LINESEG_H_

#include <Eigen/Dense>

class line_segment
{
	public:
		Eigen::Vector2f a, b;

		line_segment();
		line_segment(const Eigen::Vector2f& _a, const Eigen::Vector2f& _b);
		line_segment translate(const Eigen::Vector2f& r0);
};

#endif