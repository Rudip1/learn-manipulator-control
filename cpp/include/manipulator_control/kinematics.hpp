#pragma once
/// @file kinematics.hpp
/// Forward kinematics of serial chains: Denavit-Hartenberg and product of exponentials (chapter 2,
/// 1_theory/02_forward_kinematics.md).

#include <Eigen/Core>
#include <vector>

#include "manipulator_control/transforms.hpp"

namespace manipulator_control {

enum class JointType { Revolute, Prismatic };

/// One link in standard (distal) Denavit-Hartenberg form. For a revolute joint @p theta is the constant
/// offset added to the joint variable; for a prismatic joint @p d is. Section 2.2.
struct DHLink {
    double a = 0.0;      ///< length of the common normal (along x_i)
    double alpha = 0.0;  ///< twist about x_i
    double d = 0.0;      ///< offset along z_{i-1}
    double theta = 0.0;  ///< angle about z_{i-1}
    JointType type = JointType::Revolute;
};

/// Link transform ^{i-1}T_i = Rz(theta) Tz(d) Tx(a) Rx(alpha). Eq. (2.1).
Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta);

/// A serial chain described by DH parameters, with optional base and tool transforms.
class SerialChain {
  public:
    explicit SerialChain(std::vector<DHLink> links, const Eigen::Matrix4d& base = Eigen::Matrix4d::Identity(),
                         const Eigen::Matrix4d& tool = Eigen::Matrix4d::Identity());

    /// Number of joints n.
    int dof() const { return static_cast<int>(links_.size()); }
    const std::vector<DHLink>& links() const { return links_; }
    const Eigen::Matrix4d& base() const { return base_; }
    const Eigen::Matrix4d& tool() const { return tool_; }
    void set_base(const Eigen::Matrix4d& base) { base_ = base; }
    void set_tool(const Eigen::Matrix4d& tool) { tool_ = tool; }

    /// Joint types as a vector of length n.
    std::vector<JointType> joint_types() const;

    /// ^{i-1}T_i(q_i) for link i = 1..n (argument @p i is 1-based, matching the theory). Eq. (2.1).
    Eigen::Matrix4d link_transform(int i, double qi) const;

    /// Frames T_0 = base, T_1, ..., T_n, and T_e = T_n * tool: n + 2 transforms in the base (world) frame.
    /// Joint i moves about (or along) the z axis of frame i - 1. Eq. (2.2).
    std::vector<Eigen::Matrix4d> frames(const Eigen::VectorXd& q) const;

    /// End-effector pose T_e(q). Eq. (2.2).
    Eigen::Matrix4d end_effector(const Eigen::VectorXd& q) const;

  private:
    void check_size(const Eigen::VectorXd& q) const;

    std::vector<DHLink> links_;
    Eigen::Matrix4d base_;
    Eigen::Matrix4d tool_;
};

/// Product-of-exponentials description: home pose M and space-frame screw axes S_i. Section 2.3.
struct PoeChain {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    std::vector<Vector6d> screws;  ///< space-frame screw axes, (v, w) order
};

/// Forward kinematics in space form, T(q) = exp([S_1] q_1) ... exp([S_n] q_n) M. Eq. (2.4).
Eigen::Matrix4d poe_space(const PoeChain& chain, const Eigen::VectorXd& q);

/// Forward kinematics in body form, T(q) = M exp([B_1] q_1) ... exp([B_n] q_n). Eq. (2.5).
Eigen::Matrix4d poe_body(const PoeChain& chain, const Eigen::VectorXd& q);

/// Body-frame screw axes B_i = Ad_{M^-1} S_i. Eq. (2.5).
std::vector<Vector6d> body_screws(const PoeChain& chain);

/// The same arm in product-of-exponentials form: S_i is joint i's axis (z of frame i - 1) at q = 0 and
/// M = T_e(0). Eq. (2.7).
PoeChain to_poe(const SerialChain& chain);

}  // namespace manipulator_control
