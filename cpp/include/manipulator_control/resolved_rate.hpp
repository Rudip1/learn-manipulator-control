#pragma once
/// @file resolved_rate.hpp
/// Generalised inverses and resolved-rate motion control (chapter 4, 1_theory/04_resolved_rate_control.md).

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/singularities.hpp"

namespace manipulator_control {

/// Moore-Penrose pseudoinverse by SVD. Singular values below @p tol times the largest are treated as zero.
/// Eq. (4.4).
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& J, double tol = 1e-10);

/// Damped least-squares inverse J^T (J J^T + lambda^2 I)^-1. Eq. (4.6); studied in chapter 5.
Eigen::MatrixXd dls_inverse(const Eigen::MatrixXd& J, double lambda);

/// How the task velocity is mapped to joint velocities.
enum class InverseMethod { Transpose, Pseudoinverse, DampedLeastSquares };

/// The generalised inverse J^# selected by @p method (J^T, J^+, or the DLS inverse with damping @p lambda).
Eigen::MatrixXd generalized_inverse(const Eigen::MatrixXd& J, InverseMethod method, double lambda = 0.1);

/// Which part of the end-effector pose is controlled.
enum class TaskSpace {
    PlanarPosition,  ///< (x, y): rows 0-1 of the geometric Jacobian
    Position,        ///< (x, y, z): rows 0-2
    Pose             ///< position and orientation: all six rows
};

/// Task dimension m of a task space (2, 3 or 6).
int task_dimension(TaskSpace space);

/// Orientation error e_O = log(R_d R^T): the rotation, as axis times angle in the base frame, that takes R to
/// R_d. Eq. (4.10).
Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R);

/// Task error sigma_d - sigma for the end-effector pose @p T and desired pose @p T_desired (position rows are
/// p_d - p, orientation rows are orientation_error). Eqs. (4.2), (4.10).
Eigen::VectorXd task_error(TaskSpace space, const Eigen::Matrix4d& T_desired, const Eigen::Matrix4d& T);

/// Rows of the 6 x n geometric Jacobian that belong to the task space.
Eigen::MatrixXd task_jacobian(TaskSpace space, const Eigen::MatrixXd& J);

/// Settings of the resolved-rate controller and of the Euler integration used to simulate it.
struct ResolvedRateConfig {
    InverseMethod method = InverseMethod::Pseudoinverse;
    TaskSpace space = TaskSpace::Position;
    double gain = 1.0;     ///< K = gain * I
    double damping = 0.1;  ///< lambda of the DLS inverse (lambda_max for a variable schedule)
    double dt = 0.01;      ///< control period [s]
    int steps = 1000;      ///< number of control periods to simulate
    /// Per-joint speed limit [rad/s or m/s]; the command is scaled down uniformly to respect it.
    double max_joint_speed = std::numeric_limits<double>::infinity();
    /// How the DLS damping varies with the configuration (chapter 5, eqs. 5.8-5.9).
    DampingSchedule damping_schedule = DampingSchedule::Constant;
    double damping_threshold = 0.05;  ///< w0 or eps of the variable damping schedule
};

/// One control step: qdot = J^# (K e + feedforward), with e = task_error(...). Eq. (4.3). For the DLS inverse
/// the damping follows config.damping_schedule. The result is scaled down uniformly if any joint would exceed
/// max_joint_speed.
Eigen::VectorXd resolved_rate_step(const SerialChain& chain, const Eigen::VectorXd& q,
                                   const Eigen::Matrix4d& T_desired, const Eigen::VectorXd& feedforward,
                                   const ResolvedRateConfig& config);

/// Logged result of a simulation with N = steps + 1 samples.
struct Trajectory {
    Eigen::VectorXd t;            ///< N times [s]
    Eigen::MatrixXd q;            ///< N x n joint positions
    Eigen::MatrixXd qdot;         ///< N x n joint velocities commanded at each sample (last row zero)
    Eigen::MatrixXd ee_position;  ///< N x 3 end-effector positions
    Eigen::MatrixXd error;        ///< N x m task errors
    Eigen::VectorXd error_norm;   ///< N norms of the task error
};

/// Simulate resolved-rate control to a fixed desired pose: q_{k+1} = q_k + dt * qdot_k. Sections 4.6, 4.8.
Trajectory simulate_resolved_rate(const SerialChain& chain, const Eigen::VectorXd& q0,
                                  const Eigen::Matrix4d& T_desired, const ResolvedRateConfig& config);

/// Simulate tracking of a position reference: row k of @p positions (steps x m, m = 2 or 3 for the planar or
/// spatial position task) is sigma_d at step k and row k of @p velocities is its feedforward velocity.
/// Section 4.5.
Trajectory simulate_position_tracking(const SerialChain& chain, const Eigen::VectorXd& q0,
                                      const Eigen::MatrixXd& positions, const Eigen::MatrixXd& velocities,
                                      const ResolvedRateConfig& config);

/// Simulate tracking of a full-pose reference: @p poses[k] is the desired end-effector pose at step k and row
/// k of
/// @p twists (steps x 6) its feedforward velocity (linear velocity of the desired origin, angular velocity),
/// both in the base frame. The task space is forced to Pose. Section 4.5.
Trajectory simulate_pose_tracking(const SerialChain& chain, const Eigen::VectorXd& q0,
                                  const std::vector<Eigen::Matrix4d>& poses, const Eigen::MatrixXd& twists,
                                  const ResolvedRateConfig& config);

}  // namespace manipulator_control
