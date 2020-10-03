#ifndef RECT_H_
#define RECT_H_

#include <Eigen/Dense>

class rectangle {
    public:
        Eigen::Vector2d UL, BR;
        float w, h;

        rectangle(const Eigen::Vector2d& ul, const Eigen::Vector2d& br);
        rectangle translate(const Eigen::Vector2d& r0);      
};

#endif
