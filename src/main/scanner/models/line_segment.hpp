#ifndef LINESEG_H_
#define LINESEG_H_

#include <Eigen/Dense>

class line_segment
{
	public:
		Eigen::Vector2d a, b;

		line_segment();
		line_segment(const Eigen::Vector2d& a_, const Eigen::Vector2d& b_);
		line_segment translate(const Eigen::Vector2d& r0);
};

#endif