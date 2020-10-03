#include <models/rectangle.hpp>

        rectangle::rectangle(const Eigen::Vector2d& ul, const Eigen::Vector2d& br) {
            double w = abs(br(0) - ul(0)), h = abs(br(1) - ul(1));
            UL = ul;
            BR = br;
        }

        rectangle rectangle::translate(const Eigen::Vector2d& r0)) {            
            return rectangle(UL + r0, BR + r0);
        }
