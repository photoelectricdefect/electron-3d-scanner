#ifndef POLY_H_
#define POLY_H_

#include <models/rectangle.hpp>
#include <Eigen/Dense>
#include <models/line_segment.hpp>
#include <vector>

class polygon {
    public:
        std::vector<line_segment> sides;
        std::vector<Eigen::Vector2d> vertices;

        polygon(const std::vector<Eigen::Vector2d>& vertices_);
        rectangle frame();
		polygon translate(const Eigen::Vector2d& r0);
        void print_sides();
};

#endif