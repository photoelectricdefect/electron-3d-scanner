#include "sphere_fitting.hpp"
#include "alglib/cpp/src/optimization.h"
#include "iostream"

void sphere_func(const alglib::real_1d_array &u, const alglib::real_1d_array &x, double &func, void *ptr) 
{
    // this callback calculates f(x,un)=A*exp(-0.5*(x-mu)^2/sigma^2)
    // where x is a position on X-axis and un is adjustable parameter
    int stride = *((int*)ptr);
    func=0;

    for (int i = 0; i < stride; i++)
    {
        auto delta=x[i]-u[i];
        func+=pow(delta,2);
    }

    func+=-pow(u[stride],2);
}
void sphere_grad(const alglib::real_1d_array &u, const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr) 
{
    // this callback calculates f(x,un)=A*exp(-0.5*(x-mu)^2/sigma^2) and gradient G={df/dc[i]}
    // where x is a position on X-axis and u is adjustable parameter.
    int stride = *((int*)ptr);
    func=0;

    for (int i = 0; i < stride; i++)
    {
        auto delta=x[i]-u[i];
        func+=pow(delta,2);
        grad[i]=-2*delta;
    }

    grad[stride]=-2*u[stride];
    func+=-pow(u[stride],2);
}

int sphere_fitting::fit_lev_marq(const std::vector<double>& X,int ndim,std::vector<double>& center,double& r,alglib::lsfitreport& rep,int maxit,double tol) {
    int npoints=X.size()/ndim;
    alglib::real_2d_array x_;
    x_.setlength(npoints,ndim);
    x_.setcontent(npoints,ndim,X.data());
    // std::cout<<"x_: "<<x_.tostring(0)<<std::endl;
    alglib::real_1d_array f;
    f.setlength(npoints);
    auto zeros=std::vector<double>(npoints,0);
    f.setcontent(npoints,zeros.data());
    alglib::real_1d_array u;

    int nparams=center.size()+1;
    u.setlength(nparams);
    std::vector<double> params(nparams);
    
    for (size_t i = 0; i < center.size(); i++)
    {
        params[i]=center[i];
    }

    params[params.size()-1]=r;

    u.setlength(params.size());
    u.setcontent(params.size(),params.data());    
    alglib::ae_int_t maxits=maxit;
    alglib::ae_int_t info;
    alglib::lsfitstate state;
    // alglib::lsfitreport rep;
    lsfitcreatefg(x_, f, u, true, state);
    lsfitsetcond(state, tol, maxits);
    alglib::lsfitfit(state, sphere_func, sphere_grad, NULL, (void*)(&ndim));
    lsfitresults(state, info, u, rep);
    
    for(size_t i = 0; i < center.size(); i++)
    {
        center[i]=u[i];
    }

    r=u[params.size()-1];

    return int(info);
}