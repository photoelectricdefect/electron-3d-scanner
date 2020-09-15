#ifndef POLY_H_
#define POLY_H_

#include <models/rectangle.hpp>
#include <Eigen/Dense>

namespace scanner {
class polygon {
    public:
        std::vector<line_segment> sides;
        std::vector<Eigen::Vector2f> vertices;

        polygon(const std::vector<Eigen::Vector2f>& _vertices);
        rectangle frame();
		polygon translate(const Eigen::Vector2f& r0);
};
}

#endif