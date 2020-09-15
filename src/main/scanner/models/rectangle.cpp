#include <models/rectangle.hpp>

namespace scanner {
        rectangle::rectangle(const Eigen::Vector2f& ul, const Eigen::Vector2f& br) {
            float w = abs(br(0) - ul(0)), h = abs(br(1) - ul(1));
            UL = ul;
            BR = br;
        }

        rectangle rectangle::translate(const Eigen::Vector2f& r0)) {            
            return rectangle(UL + r0, BR + r0);
        }
}
