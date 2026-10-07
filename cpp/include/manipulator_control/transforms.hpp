#pragma once
/// @file transforms.hpp
/// Rotations, homogeneous transforms and twists (chapter 1, 1_theory/01_rigid_body_transforms.md).
///
/// Twists are ordered linear part first: V = (v, w). This matches the rows of the geometric Jacobian used in
/// the rest of the library.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace manipulator_control {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Skew-symmetric matrix [w]x such that [w]x u = w x u. Eq. (1.3).
Eigen::Matrix3d skew(const Eigen::Vector3d& w);

/// Inverse of skew(): the vector of the skew-symmetric part of @p S, i.e. 0.5 * (S - S^T) mapped to R^3.
Eigen::Vector3d unskew(const Eigen::Matrix3d& S);

/// Elementary rotation about the x axis by @p angle. Eq. (1.2).
Eigen::Matrix3d rot_x(double angle);
/// Elementary rotation about the y axis by @p angle. Eq. (1.2).
Eigen::Matrix3d rot_y(double angle);
/// Elementary rotation about the z axis by @p angle. Eq. (1.2).
Eigen::Matrix3d rot_z(double angle);

/// True when @p R is orthonormal with determinant +1 to within @p tol.
bool is_rotation(const Eigen::Matrix3d& R, double tol = 1e-9);

/// Exponential map on SO(3): rotation by |phi| about phi/|phi| (Rodrigues' formula, eq. (1.5)).
Eigen::Matrix3d exp_so3(const Eigen::Vector3d& phi);

/// Logarithm on SO(3): exponential coordinates phi with |phi| in [0, pi] and exp_so3(phi) == R. Eq. (1.6),
/// including the special cases theta = 0 and theta = pi.
Eigen::Vector3d log_so3(const Eigen::Matrix3d& R);

/// Rotation from roll-pitch-yaw angles, R = Rz(yaw) Ry(pitch) Rx(roll). Eq. (1.7).
Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d& rpy);

/// Roll-pitch-yaw angles (roll, pitch, yaw) of @p R, pitch in [-pi/2, pi/2]. At the representation
/// singularity pitch = +-pi/2 only yaw -+ roll is defined; roll is then set to zero.
Eigen::Vector3d rpy_from_rotation(const Eigen::Matrix3d& R);

/// Homogeneous transform with rotation @p R and translation @p p. Eq. (1.8).
Eigen::Matrix4d make_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);

/// Pure translation by @p p.
Eigen::Matrix4d translation(const Eigen::Vector3d& p);

/// Inverse of a homogeneous transform using R^T, without a general matrix inverse. Eq. (1.9).
Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T);

/// 4x4 se(3) matrix [V] of a twist V = (v, w). Eq. (1.10).
Eigen::Matrix4d twist_hat(const Vector6d& V);

/// Inverse of twist_hat().
Vector6d twist_vee(const Eigen::Matrix4d& V_hat);

/// Adjoint of T for (v, w)-ordered twists: Ad_T = [R, [p]x R; 0, R]. Eq. (1.11).
Matrix6d adjoint(const Eigen::Matrix4d& T);

/// Screw axis through point @p q with unit direction @p w_hat and pitch @p h. Eq. (1.12).
Vector6d screw_axis(const Eigen::Vector3d& q, const Eigen::Vector3d& w_hat, double h = 0.0);

/// Exponential map on SE(3) of exponential coordinates xi = S theta = (v theta, w theta). Eq. (1.13).
Eigen::Matrix4d exp_se3(const Vector6d& xi);

/// Logarithm on SE(3): exponential coordinates xi with exp_se3(xi) == T. Eq. (1.14).
Vector6d log_se3(const Eigen::Matrix4d& T);

}  // namespace manipulator_control
