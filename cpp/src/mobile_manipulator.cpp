#include "manipulator_control/mobile_manipulator.hpp"

#include <cmath>
#include <stdexcept>

#include "manipulator_control/jacobian.hpp"

namespace manipulator_control {

Eigen::Matrix4d base_transform(const Eigen::Vector3d& eta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const double c = std::cos(eta(2)), s = std::sin(eta(2));
    T(0, 0) = c;
    T(0, 1) = -s;
    T(1, 0) = s;
    T(1, 1) = c;
    T(0, 3) = eta(0);
    T(1, 3) = eta(1);
    return T;  // eq. (9.3)
}

Eigen::Vector3d integrate_unicycle(const Eigen::Vector3d& eta, double omega, double v, double dt) {
    const double psi = eta(2), dpsi = omega * dt;
    Eigen::Vector3d out = eta;
    if (std::abs(dpsi) < 1e-9) {  // straight segment (limit of eq. 9.6)
        out(0) += v * dt * std::cos(psi + 0.5 * dpsi);
        out(1) += v * dt * std::sin(psi + 0.5 * dpsi);
    } else {  // arc of radius v / omega, eq. (9.6)
        out(0) += v / omega * (std::sin(psi + dpsi) - std::sin(psi));
        out(1) -= v / omega * (std::cos(psi + dpsi) - std::cos(psi));
    }
    out(2) = psi + dpsi;
    return out;
}

MobileManipulator::MobileManipulator(SerialChain arm, const Eigen::Matrix4d& mount)
    : arm_(std::move(arm)), mount_(mount) {}

void MobileManipulator::check_size(const Eigen::VectorXd& q) const {
    if (q.size() != state_size()) throw std::invalid_argument("MobileManipulator: q must have size 3 + n");
}

Eigen::Matrix4d MobileManipulator::frame_pose(const Eigen::VectorXd& q, int k) const {
    check_size(q);
    const Eigen::Matrix4d T_base = base_transform(q.head<3>());
    if (k == 0) return T_base;
    const std::vector<Eigen::Matrix4d> F = arm_.frames(q.tail(arm_.dof()));
    if (k < 0) return T_base * mount_ * F.back();
    if (k > arm_.dof() + 1) throw std::out_of_range("MobileManipulator: frame index out of range");
    return T_base * mount_ * F[size_t(k - 1)];
}

Eigen::MatrixXd MobileManipulator::frame_jacobian(const Eigen::VectorXd& q, int k) const {
    check_size(q);
    const int n = arm_.dof();
    const double psi = q(2);
    const Eigen::Vector3d p = frame_pose(q, k).block<3, 1>(0, 3);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 2 + n);
    // omega: the whole robot turns about the vertical axis through the base origin, eq. (9.5).
    J(0, 0) = -(p.y() - q(1));
    J(1, 0) = p.x() - q(0);
    J(5, 0) = 1.0;
    // v: translation along the heading.
    J(0, 1) = std::cos(psi);
    J(1, 1) = std::sin(psi);
    if (k == 0) return J;
    // Arm joints: the arm's own Jacobian, rotated from the arm frame into the world frame.
    const int arm_frame = k < 0 ? n + 1 : k - 1;
    const Eigen::MatrixXd J_arm = link_jacobian(arm_, q.tail(n), arm_frame);
    const Eigen::Matrix3d R = (base_transform(q.head<3>()) * mount_).topLeftCorner<3, 3>();
    J.block(0, 2, 3, n) = R * J_arm.topRows(3);
    J.block(3, 2, 3, n) = R * J_arm.bottomRows(3);
    return J;
}

int MobileManipulator::state_index(int j) const {
    if (j == 0) return 2;
    if (j == 1) return -1;
    if (j >= 2 && j < 2 + arm_.dof()) return 3 + (j - 2);
    throw std::out_of_range("MobileManipulator: control index out of range");
}

Eigen::VectorXd MobileManipulator::integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const {
    check_size(q);
    if (u.size() != control_size()) throw std::invalid_argument("MobileManipulator: u must have size 2 + n");
    Eigen::VectorXd out(q.size());
    out.head<3>() = integrate_unicycle(q.head<3>(), u(0), u(1), dt);
    out.tail(arm_.dof()) = q.tail(arm_.dof()) + dt * u.tail(arm_.dof());
    return out;
}

}  // namespace manipulator_control
