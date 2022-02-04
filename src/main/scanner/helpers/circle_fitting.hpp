#ifndef CIRCLEFIT_H_
#define CIRCLEFIT_H_
#include <vector>
#include <Eigen/Dense>
#include "alglib/cpp/src/interpolation.h"

class circle_fitting
{
private:
public:
    static int fit_lev_marq(const std::vector<double>& X,int ndim,std::vector<double>& origin,std::vector<double>& direction,double& r,alglib::lsfitreport& rep,int maxit=1000,double tol=1e-6);
};


#endif