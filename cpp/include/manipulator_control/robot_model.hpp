#pragma once
/// @file robot_model.hpp
/// The kinematic interface used by task-priority control (chapters 7-9): a configuration q, a vector of
/// commanded velocities u, frame poses and the Jacobians that map u to frame twists. For a fixed arm u = qdot;
/// for a mobile manipulator (chapter 9) u holds the base speeds and the joint rates.

#include <Eigen/Core>
#include <memory>

#include "manipulator_control/kinematics.hpp"

namespace manipulator_control {

class RobotModel {
  public:
    virtual ~RobotModel() = default;

    /// Size of the configuration q.
    virtual int state_size() const = 0;

    /// Size of the commanded velocity u (the columns of every Jacobian).
    virtual int control_size() const = 0;

    /// Pose of frame @p k in the world frame. k = -1 is the end effector; the meaning of k >= 0 is set by the
    /// model (DH frame index for an arm).
    virtual Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q, int k) const = 0;

    /// 6 x control_size Jacobian of frame @p k: rows (v, w) in the world frame, columns per entry of u.
    virtual Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q, int k) const = 0;

    /// Index in q of the position driven by entry @p j of u (a joint angle or offset), or -1 when u_j does
    /// not integrate to a single coordinate (a base's forward speed).
    virtual int state_index(int j) const = 0;

    /// Configuration after applying u for dt seconds.
    virtual Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const = 0;
};

/// A fixed-base serial arm: q = joint positions, u = qdot, frames are the DH frames 0..n and k = -1 (or n + 1)
/// is the end effector.
class SerialArmModel : public RobotModel {
  public:
    explicit SerialArmModel(SerialChain chain) : chain_(std::move(chain)) {}

    const SerialChain& chain() const { return chain_; }
    int state_size() const override { return chain_.dof(); }
    int control_size() const override { return chain_.dof(); }
    Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q, int k) const override;
    Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q, int k) const override;
    int state_index(int j) const override { return j; }
    Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const override {
        return q + dt * u;
    }

  private:
    SerialChain chain_;
};

}  // namespace manipulator_control
