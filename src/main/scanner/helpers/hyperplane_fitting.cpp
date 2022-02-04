#include "hyperplane_fitting.hpp"

void hyperplane_fitting::fit_svd(const std::vector<Eigen::Vector3d>& points, Eigen::Hyperplane<double, 3>& plane)
{
    Eigen::MatrixXd A(3,points.size());

    for (int i = 0; i < points.size(); i++) {
        A.col(i) = points[i];
    }

    Eigen::Vector3d c0 = A.rowwise().mean();

    for (int i = 0; i < points.size(); i++) {
        A.col(i)-=c0;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV | Eigen::ComputeThinU);
    svd.computeV();
    Eigen::Vector3d n = svd.matrixU().col(A.rows() - 1);
    n.normalize();
    plane = Eigen::Hyperplane<double, 3>(n, c0);
}

void hyperplane_fitting::fit_eigen(const std::vector<Eigen::Vector3d>& points, Eigen::Hyperplane<double, 3>& plane)
{
    Eigen::MatrixXd A(3,points.size());

    for (int i = 0; i < points.size(); i++) {
        A.col(i) = points[i];
    }

    Eigen::Vector3d c0 = A.rowwise().mean();

    for (int i = 0; i < points.size(); i++) {
        A.col(i)-=c0;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    Eigen::Matrix3d AAT = A*A.transpose();
    es.compute(AAT);
    Eigen::Vector3d n = es.eigenvectors().col(0);
    n.normalize();
    plane = Eigen::Hyperplane<double, 3>(n, c0);
}
