#pragma once
/// @file mobile_manipulator.hpp
/// A differential-drive (unicycle) base carrying a serial arm, as one RobotModel for task-priority control
/// (chapter 9, 1_theory/09_mobile_manipulators.md).

#include <Eigen/Core>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/robot_model.hpp"

namespace manipulator_control {

/// Pose of a planar base eta = (x, y, psi) as a 4 x 4 transform (rotation psi about z, height 0). Eq. (9.3).
Eigen::Matrix4d base_transform(const Eigen::Vector3d& eta);

/// Unicycle motion with constant forward speed @p v and turn rate @p omega for @p dt seconds, integrated
/// exactly (an arc of a circle, or a straight segment for omega = 0). Eq. (9.6).
Eigen::Vector3d integrate_unicycle(const Eigen::Vector3d& eta, double omega, double v, double dt);

/// Mobile manipulator: configuration q = (x, y, psi, q_arm) of size 3 + n, commanded velocity
/// u = zeta = (omega, v, qdot_arm) of size 2 + n (eq. 9.4). The arm is mounted on the base by the constant
/// transform @p mount (base frame -> frame 0 of the arm chain; the chain's own base transform is applied too).
///
/// Frames: -1 is the end effector, 0 the base frame, and k = 1..n + 1 the arm's DH frames 0..n.
class MobileManipulator : public RobotModel {
  public:
    MobileManipulator(SerialChain arm, const Eigen::Matrix4d& mount);

    const SerialChain& arm() const { return arm_; }
    const Eigen::Matrix4d& mount() const { return mount_; }
    int arm_dof() const { return arm_.dof(); }

    int state_size() const override { return 3 + arm_.dof(); }
    int control_size() const override { return 2 + arm_.dof(); }
    Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q, int k) const override;
    /// Columns: omega (rotation of the whole robot about the base's vertical axis), v (translation along the
    /// base's heading), then the arm joints. Eq. (9.5).
    Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q, int k) const override;
    /// omega drives psi (index 2); v has no single coordinate (-1); arm joint i drives index 3 + i.
    int state_index(int j) const override;
    /// Exact arc for the base (eq. 9.6), Euler for the arm joints.
    Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const override;

  private:
    void check_size(const Eigen::VectorXd& q) const;

    SerialChain arm_;
    Eigen::Matrix4d mount_;
};

}  // namespace manipulator_control
