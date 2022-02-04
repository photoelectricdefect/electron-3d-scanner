#ifndef FITPLANE_H_
#define FITPLANE_H_
#include <vector>
#include <Eigen/Dense>

class hyperplane_fitting
{
private:
public:
    static void fit_svd(const std::vector<Eigen::Vector3d>& points, Eigen::Hyperplane<double, 3>& plane);
    static void fit_eigen(const std::vector<Eigen::Vector3d>& points, Eigen::Hyperplane<double, 3>& plane);
};


#endif