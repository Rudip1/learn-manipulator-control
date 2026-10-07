#pragma once
/// @file jacobian.hpp
/// Geometric, space, body and analytic Jacobians and their finite-difference check (chapter 3,
/// 1_theory/03_jacobian.md). Rows are ordered (v, w): linear velocity first.

#include <Eigen/Core>
#include <vector>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/transforms.hpp"

namespace manipulator_control {

/// Geometric Jacobian of the origin of frames[k] from a list of frames T_0..T_n, T_e (as returned by
/// SerialChain::frames) and the joint types. Joint i (1-based) contributes column i if i <= k; its axis is z
/// of frames[i-1]. Columns of joints beyond frame k are zero. Velocities are expressed in the base (world)
/// frame. Eqs. (3.2)-(3.4).
Eigen::MatrixXd geometric_jacobian(const std::vector<Eigen::Matrix4d>& frames,
                                   const std::vector<JointType>& types, int k);

/// End-effector geometric Jacobian, 6 x n. Eq. (3.3).
Eigen::MatrixXd geometric_jacobian(const SerialChain& chain, const Eigen::VectorXd& q);

/// Geometric Jacobian of DH frame k (0..n) or of the end effector (k = n + 1). Eq. (3.4).
Eigen::MatrixXd link_jacobian(const SerialChain& chain, const Eigen::VectorXd& q, int k);

/// Space Jacobian of a product-of-exponentials chain: column i is Ad_{e^[S1]q1...e^[S_{i-1}]q_{i-1}} S_i.
/// Eq. (3.5).
Eigen::MatrixXd space_jacobian(const PoeChain& chain, const Eigen::VectorXd& q);

/// Body Jacobian J_b = Ad_{T^-1} J_s. Eq. (3.6).
Eigen::MatrixXd body_jacobian(const PoeChain& chain, const Eigen::VectorXd& q);

/// Map from roll-pitch-yaw rates to angular velocity, w = T(rpy) d(rpy)/dt, for R = Rz(yaw) Ry(pitch)
/// Rx(roll). det T = cos(pitch). Eq. (3.8).
Eigen::Matrix3d rpy_rate_matrix(const Eigen::Vector3d& rpy);

/// Pose of a frame as (position, roll, pitch, yaw), the minimal parametrisation used by the analytic
/// Jacobian.
Vector6d pose_rpy(const Eigen::Matrix4d& T);

/// Analytic Jacobian of x = (p, roll, pitch, yaw) for the end effector: blockdiag(I, T(rpy)^-1) J. Eq. (3.9).
/// Undefined at pitch = +-pi/2.
Eigen::MatrixXd analytic_jacobian(const SerialChain& chain, const Eigen::VectorXd& q);

/// Geometric Jacobian of frame k by central differences: position rows from (p(q + h e_i) - p(q - h e_i)) /
/// 2h, angular rows from log(R(q + h e_i) R(q - h e_i)^T) / 2h. Eq. (3.10). k = -1 means the end effector.
Eigen::MatrixXd numerical_jacobian(const SerialChain& chain, const Eigen::VectorXd& q, double h = 1e-6,
                                   int k = -1);

/// Analytic Jacobian by central differences of pose_rpy (wrapping the angle differences). Eq. (3.10).
Eigen::MatrixXd numerical_analytic_jacobian(const SerialChain& chain, const Eigen::VectorXd& q,
                                            double h = 1e-6);

}  // namespace manipulator_control
