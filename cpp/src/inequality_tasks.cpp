#include "manipulator_control/inequality_tasks.hpp"

#include <stdexcept>

namespace manipulator_control {

int next_activation(int state, double sigma, double sigma_min, double sigma_max, double alpha, double delta) {
    // Eq. (8.3): switch on within alpha of a bound, off once delta away from it.
    if (state == 0) {
        if (sigma >= sigma_max - alpha) return -1;
        if (sigma <= sigma_min + alpha) return +1;
        return 0;
    }
    if (state == -1 && sigma <= sigma_max - delta) return 0;
    if (state == +1 && sigma >= sigma_min + delta) return 0;
    return state;
}

SetBasedTask::SetBasedTask(std::string name, double sigma_min, double sigma_max, double alpha, double delta)
    : Task(std::move(name), 1), sigma_min_(sigma_min), sigma_max_(sigma_max), alpha_(alpha), delta_(delta) {
    if (!(sigma_min < sigma_max)) throw std::invalid_argument("SetBasedTask: need sigma_min < sigma_max");
    if (alpha < 0.0 || delta < alpha) throw std::invalid_argument("SetBasedTask: need 0 <= alpha <= delta");
}

void SetBasedTask::update(const RobotModel& model, const Eigen::VectorXd& q) {
    Eigen::RowVectorXd J;
    sigma_ = compute(model, q, J);
    J_ = J;
    activation_ = next_activation(activation_, sigma_, sigma_min_, sigma_max_, alpha_, delta_);
    // Eq. (8.4): while active, push sigma away from the bound at the constant rate K (error = +-1).
    error_(0) = static_cast<double>(activation_);
}

JointLimitTask::JointLimitTask(std::string name, int control_index, double q_min, double q_max, double alpha,
                               double delta)
    : SetBasedTask(std::move(name), q_min, q_max, alpha, delta), control_index_(control_index) {}

double JointLimitTask::compute(const RobotModel& model, const Eigen::VectorXd& q, Eigen::RowVectorXd& J) const {
    const int s = model.state_index(control_index_);
    if (s < 0) throw std::invalid_argument("JointLimitTask: this entry of u has no position");
    J = Eigen::RowVectorXd::Zero(model.control_size());
    J(control_index_) = 1.0;  // eq. (8.5)
    return q(s);
}

ObstacleTask::ObstacleTask(std::string name, int frame, const Eigen::Vector3d& center, double radius,
                           double alpha, double delta, bool planar)
    : SetBasedTask(std::move(name), radius, std::numeric_limits<double>::infinity(), alpha, delta),
      frame_(frame),
      center_(center),
      planar_(planar) {}

double ObstacleTask::compute(const RobotModel& model, const Eigen::VectorXd& q, Eigen::RowVectorXd& J) const {
    Eigen::Vector3d d = model.frame_pose(q, frame_).block<3, 1>(0, 3) - center_;
    if (planar_) d.z() = 0.0;
    const double dist = d.norm();
    if (dist < 1e-12) throw std::runtime_error("ObstacleTask: frame origin at the obstacle centre");
    const Eigen::MatrixXd Jp = model.frame_jacobian(q, frame_).topRows(3);
    J = (d.transpose() / dist) * Jp;  // eq. (8.6)
    return dist;
}

}  // namespace manipulator_control
