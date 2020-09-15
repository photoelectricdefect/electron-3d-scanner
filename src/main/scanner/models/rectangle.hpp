#ifndef RECT_H_
#define RECT_H_

#include <Eigen/Dense>

class rectangle {
    public:
        Eigen::Vector2f UL, BR;
        float w, h;

        rectangle(const Eigen::Vector2f& ul, const Eigen::Vector2f& br);
        rectangle translate(const Eigen::Vector2f& r0);      
};

#endif
