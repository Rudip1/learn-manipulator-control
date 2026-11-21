#pragma once
/// @file redundancy.hpp
/// Kinematic redundancy: null-space projectors, weighted pseudoinverses and secondary objectives resolved by
/// gradient projection (chapter 6, 1_theory/06_redundancy.md).

#include <Eigen/Core>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/resolved_rate.hpp"

namespace manipulator_control {

/// Null-space projector N = I - J^+ J (n x n). Singular values below @p tol times the largest are treated as
/// zero, so N also spans the directions lost at a singularity. Eq. (6.3).
Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J, double tol = 1e-10);

/// Projector I - J^# J for any generalised inverse J^# (for the DLS inverse this is not an exact projector;
/// eq. 6.12, section 6.5).
Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J, const Eigen::MatrixXd& J_inverse);

/// Weighted pseudoinverse J_W^+ = W^-1 J^T (J W^-1 J^T)^-1: the joint velocity of smallest weighted norm
/// qdot^T W qdot that achieves J qdot = v. Computed as L^-T (J L^-T)^+ with W = L L^T, so it is also defined
/// for a rank-deficient J. @p W must be symmetric positive definite. Eq. (6.6).
Eigen::MatrixXd weighted_pseudoinverse(const Eigen::MatrixXd& J, const Eigen::MatrixXd& W, double tol = 1e-10);

/// Joint-centering cost H(q) = 1/(2n) sum_i ((q_i - qbar_i) / (qmax_i - qmin_i))^2 with qbar the middle of
/// the range (Liegeois). Eq. (6.9).
double joint_centering_cost(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max);

/// Gradient of joint_centering_cost. Eq. (6.10).
Eigen::VectorXd joint_centering_gradient(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min,
                                         const Eigen::VectorXd& q_max);

/// Manipulability w(q) = sqrt(det(J J^T)) of the task Jacobian (rows of @p space). Eq. (5.2).
double task_manipulability(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space);

/// Gradient of task_manipulability by central differences with step @p h. Eq. (6.11).
Eigen::VectorXd manipulability_gradient(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                        double h = 1e-6);

/// Secondary objective optimised in the null space of the task.
enum class SecondaryObjective {
    None,            ///< minimum-norm solution, qdot = J^# v
    JointCentering,  ///< minimise H(q) of eq. (6.9): z = -k0 grad H
    Manipulability,  ///< maximise w(q): z = +k0 grad w
};

/// Settings of the null-space term of the redundancy resolution.
struct RedundancyConfig {
    SecondaryObjective objective = SecondaryObjective::None;
    double k0 = 1.0;    ///< step size of the gradient projection, eq. (6.8)
    Eigen::VectorXd q_min;  ///< joint ranges, needed for JointCentering
    Eigen::VectorXd q_max;
    /// false drops the projector and adds z directly (wrong; section 6.7).
    bool project = true;
};

/// Value of the secondary objective at q (H for JointCentering, w for Manipulability, 0 for None).
double secondary_objective_value(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                 const RedundancyConfig& redundancy);

/// Null-space velocity z of the secondary objective at q: -k0 grad H or +k0 grad w. Eq. (6.8).
Eigen::VectorXd secondary_velocity(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                   const RedundancyConfig& redundancy);

/// One control step of gradient projection: qdot = J^# (K e + feedforward) + (I - J^# J) z. Eq. (6.8). The
/// inverse J^# follows config.method (and its damping schedule); the result is scaled down uniformly if any
/// joint would exceed config.max_joint_speed.
Eigen::VectorXd redundancy_step(const SerialChain& chain, const Eigen::VectorXd& q, const Eigen::Matrix4d& T_desired,
                                const Eigen::VectorXd& feedforward, const ResolvedRateConfig& config,
                                const RedundancyConfig& redundancy);

/// Simulated trajectory plus the secondary objective at every sample.
struct RedundancyTrajectory {
    Trajectory trajectory;
    Eigen::VectorXd objective;  ///< N values of secondary_objective_value
};

/// Simulate gradient projection to a fixed desired pose: q_{k+1} = q_k + dt * qdot_k. Section 6.4.
RedundancyTrajectory simulate_redundancy(const SerialChain& chain, const Eigen::VectorXd& q0,
                                         const Eigen::Matrix4d& T_desired, const ResolvedRateConfig& config,
                                         const RedundancyConfig& redundancy);

}  // namespace manipulator_control
