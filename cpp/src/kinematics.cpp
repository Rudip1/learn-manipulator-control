#include "manipulator_control/kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulator_control {

Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta) {
    // Eq. (2.1): Rz(theta) Tz(d) Tx(a) Rx(alpha) multiplied out.
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    Eigen::Matrix4d A;
    A << ct, -st * ca, st * sa, a * ct,  //
        st, ct * ca, -ct * sa, a * st,   //
        0.0, sa, ca, d,                  //
        0.0, 0.0, 0.0, 1.0;
    return A;
}

SerialChain::SerialChain(std::vector<DHLink> links, const Eigen::Matrix4d& base, const Eigen::Matrix4d& tool)
    : links_(std::move(links)), base_(base), tool_(tool) {}

std::vector<JointType> SerialChain::joint_types() const {
    std::vector<JointType> types;
    types.reserve(links_.size());
    for (const auto& l : links_) types.push_back(l.type);
    return types;
}

Eigen::Matrix4d SerialChain::link_transform(int i, double qi) const {
    if (i < 1 || i > dof()) throw std::out_of_range("link index " + std::to_string(i) + " outside 1..n");
    const DHLink& l = links_[static_cast<size_t>(i - 1)];
    if (l.type == JointType::Revolute) return dh_transform(l.a, l.alpha, l.d, l.theta + qi);
    return dh_transform(l.a, l.alpha, l.d + qi, l.theta);
}

std::vector<Eigen::Matrix4d> SerialChain::frames(const Eigen::VectorXd& q) const {
    check_size(q);
    std::vector<Eigen::Matrix4d> T;
    T.reserve(links_.size() + 2);
    T.push_back(base_);
    for (int i = 1; i <= dof(); ++i) T.push_back(T.back() * link_transform(i, q(i - 1)));  // eq. (2.2)
    T.push_back(T.back() * tool_);
    return T;
}

Eigen::Matrix4d SerialChain::end_effector(const Eigen::VectorXd& q) const { return frames(q).back(); }

void SerialChain::check_size(const Eigen::VectorXd& q) const {
    if (q.size() != dof()) {
        throw std::invalid_argument("expected " + std::to_string(dof()) + " joint values, got " +
                                    std::to_string(q.size()));
    }
}

Eigen::Matrix4d poe_space(const PoeChain& chain, const Eigen::VectorXd& q) {
    if (q.size() != static_cast<Eigen::Index>(chain.screws.size()))
        throw std::invalid_argument("poe_space: q has the wrong size");
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < chain.screws.size(); ++i) T = T * exp_se3(chain.screws[i] * q(Eigen::Index(i)));
    return T * chain.M;  // eq. (2.4)
}

std::vector<Vector6d> body_screws(const PoeChain& chain) {
    const Matrix6d Ad_Minv = adjoint(inverse_transform(chain.M));
    std::vector<Vector6d> B;
    B.reserve(chain.screws.size());
    for (const auto& S : chain.screws) B.push_back(Ad_Minv * S);  // eq. (2.5)
    return B;
}

Eigen::Matrix4d poe_body(const PoeChain& chain, const Eigen::VectorXd& q) {
    if (q.size() != static_cast<Eigen::Index>(chain.screws.size()))
        throw std::invalid_argument("poe_body: q has the wrong size");
    const std::vector<Vector6d> B = body_screws(chain);
    Eigen::Matrix4d T = chain.M;
    for (size_t i = 0; i < B.size(); ++i) T = T * exp_se3(B[i] * q(Eigen::Index(i)));
    return T;  // eq. (2.5)
}

PoeChain to_poe(const SerialChain& chain) {
    // Eq. (2.7): the axis of joint i is z_{i-1} through o_{i-1}, read at the home configuration q = 0.
    const std::vector<Eigen::Matrix4d> T = chain.frames(Eigen::VectorXd::Zero(chain.dof()));
    PoeChain poe;
    poe.M = T.back();
    for (int i = 1; i <= chain.dof(); ++i) {
        const Eigen::Vector3d z = T[size_t(i - 1)].block<3, 1>(0, 2);
        const Eigen::Vector3d o = T[size_t(i - 1)].block<3, 1>(0, 3);
        Vector6d S;
        if (chain.links()[size_t(i - 1)].type == JointType::Revolute) {
            S << -z.cross(o), z;
        } else {
            S << z, Eigen::Vector3d::Zero();
        }
        poe.screws.push_back(S);
    }
    return poe;
}

}  // namespace manipulator_control
