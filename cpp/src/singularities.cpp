#include "manipulator_control/singularities.hpp"

#include <Eigen/SVD>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace manipulator_control {

Eigen::VectorXd singular_values(const Eigen::MatrixXd& J) {
    return Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues();
}

double manipulability(const Eigen::MatrixXd& J) {
    if (J.rows() > J.cols()) return 0.0;  // J J^T is rank deficient
    return singular_values(J).prod();     // eq. (5.2): product of the m singular values
}

double condition_number(const Eigen::MatrixXd& J) {
    const Eigen::VectorXd s = singular_values(J);
    if (J.rows() > J.cols() || s(s.size() - 1) == 0.0) return std::numeric_limits<double>::infinity();
    return s(0) / s(s.size() - 1);
}

Ellipsoid manipulability_ellipsoid(const Eigen::MatrixXd& J) {
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU);
    Ellipsoid e;
    e.axes = svd.matrixU();
    e.radii = Eigen::VectorXd::Zero(J.rows());
    e.radii.head(svd.singularValues().size()) = svd.singularValues();
    return e;
}

double damping_factor(const Eigen::MatrixXd& J, DampingSchedule schedule, double lambda_max,
                      double threshold) {
    switch (schedule) {
        case DampingSchedule::Constant:
            return lambda_max;
        case DampingSchedule::Manipulability: {
            const double w = manipulability(J);
            const double r = 1.0 - w / threshold;
            return w < threshold ? lambda_max * r * r : 0.0;  // eq. (5.8)
        }
        case DampingSchedule::MinSingularValue: {
            const Eigen::VectorXd s = singular_values(J);
            const double s_min = J.rows() > J.cols() ? 0.0 : s(s.size() - 1);
            if (s_min >= threshold) return 0.0;
            const double r = s_min / threshold;
            return lambda_max * std::sqrt(1.0 - r * r);  // eq. (5.9)
        }
    }
    throw std::invalid_argument("damping_factor: unknown schedule");
}

}  // namespace manipulator_control
