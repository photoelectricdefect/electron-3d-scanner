#ifndef TLSLINE_H_
#define TLSLINE_H_
#include <vector>
#include <Eigen/Dense>

class tls_line
{
private:
public:
    static void fit_eigen(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& source,Eigen::Vector3d& direction);
};


#endif