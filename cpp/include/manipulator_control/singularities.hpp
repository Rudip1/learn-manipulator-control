#pragma once
/// @file singularities.hpp
/// Singular values, manipulability and damping schedules for the damped least-squares inverse (chapter 5,
/// 1_theory/05_singularities.md).

#include <Eigen/Core>

namespace manipulator_control {

/// Singular values s_1 >= ... >= s_r of J, r = min(rows, cols).
Eigen::VectorXd singular_values(const Eigen::MatrixXd& J);

/// Yoshikawa's manipulability w = sqrt(det(J J^T)) = s_1 s_2 ... s_m (zero if J has more rows than columns).
/// Eq. (5.2).
double manipulability(const Eigen::MatrixXd& J);

/// Condition number s_1 / s_m (infinity at a singularity).
double condition_number(const Eigen::MatrixXd& J);

/// Principal axes of the velocity ellipsoid {J qdot : |qdot| <= 1}: columns of @p axes are the left singular
/// vectors u_i and @p radii the singular values s_i. Eq. (5.1).
struct Ellipsoid {
    Eigen::MatrixXd axes;
    Eigen::VectorXd radii;
};
Ellipsoid manipulability_ellipsoid(const Eigen::MatrixXd& J);

/// How the damping factor of the DLS inverse is chosen.
enum class DampingSchedule {
    Constant,          ///< lambda = lambda_max everywhere
    Manipulability,    ///< lambda = lambda_max (1 - w / w0)^2 for w < w0, else 0. Eq. (5.8)
    MinSingularValue,  ///< lambda^2 = (1 - (s_m / eps)^2) lambda_max^2 for s_m < eps, else 0. Eq. (5.9)
};

/// Damping factor lambda for Jacobian @p J. @p threshold is w0 (Manipulability) or eps (MinSingularValue) and
/// is ignored for Constant. Eqs. (5.8), (5.9).
double damping_factor(const Eigen::MatrixXd& J, DampingSchedule schedule, double lambda_max,
                      double threshold);

}  // namespace manipulator_control
