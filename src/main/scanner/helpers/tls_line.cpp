#include "tls_line.hpp"

void tls_line::fit_eigen(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& source,Eigen::Vector3d& direction)
{
    Eigen::MatrixXd A(3,points.size());

    for (int i = 0; i < points.size(); i++) {
        A.col(i) = points[i];
    }

    Eigen::Vector3d c0 = A.rowwise().mean();
    Eigen::Vector3d d(0,0,0);
    
    for (int i = 0; i < points.size(); i++) {
        auto y=A.col(i)-c0;
        A.col(i)=y;
        auto y_sq=y.squaredNorm();
        d+=Eigen::Vector3d(y_sq,y_sq,y_sq);
    }

    Eigen::Matrix3d AAT = A*A.transpose();
    Eigen::Matrix3d M;
    Eigen::Matrix3d D(d.asDiagonal());
    M=D-AAT;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    es.compute(M);
    source = c0;
    direction = es.eigenvectors().col(0);
    direction.normalize();
}
