#include "manipulator_control/tasks.hpp"

#include <stdexcept>

#include "manipulator_control/resolved_rate.hpp"

namespace manipulator_control {

Task::Task(std::string name, int dimension)
    : name_(std::move(name)),
      dimension_(dimension),
      K_(Eigen::MatrixXd::Identity(dimension, dimension)),
      feedforward_(Eigen::VectorXd::Zero(dimension)) {
    if (dimension <= 0) throw std::invalid_argument("Task: dimension must be positive");
    error_ = Eigen::VectorXd::Zero(dimension);
}

Eigen::VectorXd Task::velocity() const { return feedforward_ + K_ * error_; }  // eq. (7.2)

void Task::set_gain(double k) { K_ = k * Eigen::MatrixXd::Identity(dimension_, dimension_); }

void Task::set_gain(const Eigen::MatrixXd& K) {
    if (K.rows() != dimension_ || K.cols() != dimension_) throw std::invalid_argument("Task: K must be m x m");
    K_ = K;
}

void Task::set_feedforward(const Eigen::VectorXd& feedforward) {
    if (feedforward.size() != dimension_) throw std::invalid_argument("Task: feedforward must have size m");
    feedforward_ = feedforward;
}

FrameTask::FrameTask(std::string name, int frame, std::vector<int> components, const Eigen::Matrix4d& desired)
    : Task(std::move(name), static_cast<int>(components.size())),
      frame_(frame),
      components_(std::move(components)),
      desired_(desired) {
    for (int c : components_)
        if (c < 0 || c > 5) throw std::invalid_argument("FrameTask: components must be in 0..5");
}

void FrameTask::update(const RobotModel& model, const Eigen::VectorXd& q) {
    const Eigen::Matrix4d T = model.frame_pose(q, frame_);
    const Eigen::MatrixXd J6 = model.frame_jacobian(q, frame_);
    const Eigen::VectorXd e6 = task_error(TaskSpace::Pose, desired_, T);  // eqs. (4.2), (4.10)
    J_.resize(dimension(), J6.cols());
    for (int r = 0; r < dimension(); ++r) {
        J_.row(r) = J6.row(components_[size_t(r)]);
        error_(r) = e6(components_[size_t(r)]);
    }
}

JointTask::JointTask(std::string name, int control_index, double desired)
    : Task(std::move(name), 1), control_index_(control_index), desired_(desired) {}

void JointTask::update(const RobotModel& model, const Eigen::VectorXd& q) {
    const int s = model.state_index(control_index_);
    if (s < 0) throw std::invalid_argument("JointTask: this entry of u has no position");
    J_ = Eigen::MatrixXd::Zero(1, model.control_size());
    J_(0, control_index_) = 1.0;
    error_(0) = desired_ - q(s);
}

}  // namespace manipulator_control
