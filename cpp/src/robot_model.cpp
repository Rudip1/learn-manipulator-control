#include "manipulator_control/robot_model.hpp"

#include "manipulator_control/jacobian.hpp"

namespace manipulator_control {

Eigen::Matrix4d SerialArmModel::frame_pose(const Eigen::VectorXd& q, int k) const {
    if (k < 0) return chain_.end_effector(q);
    return chain_.frames(q).at(static_cast<size_t>(k));
}

Eigen::MatrixXd SerialArmModel::frame_jacobian(const Eigen::VectorXd& q, int k) const {
    return link_jacobian(chain_, q, k < 0 ? chain_.dof() + 1 : k);
}

}  // namespace manipulator_control
