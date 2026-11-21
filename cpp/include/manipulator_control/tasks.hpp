#pragma once
/// @file tasks.hpp
/// Tasks for the task-priority controller (chapter 7, 1_theory/07_task_priority.md). A task owns its
/// Jacobian J (m x control_size) and its error, both recomputed by update() from the robot model, and a gain
/// K. Its desired task velocity is v = feedforward + K * error.

#include <Eigen/Core>
#include <string>
#include <vector>

#include "manipulator_control/robot_model.hpp"

namespace manipulator_control {

class Task {
  public:
    Task(std::string name, int dimension);
    virtual ~Task() = default;

    const std::string& name() const { return name_; }
    int dimension() const { return dimension_; }

    /// Recompute the Jacobian and the error at configuration @p q.
    virtual void update(const RobotModel& model, const Eigen::VectorXd& q) = 0;

    /// Whether the task takes part in the hierarchy now. Equality tasks are always active; inequality tasks
    /// switch (chapter 8).
    virtual bool is_active() const { return true; }

    const Eigen::MatrixXd& jacobian() const { return J_; }
    const Eigen::VectorXd& error() const { return error_; }

    /// Desired task velocity v = feedforward + K error. Eq. (7.2).
    virtual Eigen::VectorXd velocity() const;

    void set_gain(double k);
    void set_gain(const Eigen::MatrixXd& K);
    const Eigen::MatrixXd& gain() const { return K_; }
    void set_feedforward(const Eigen::VectorXd& feedforward);
    const Eigen::VectorXd& feedforward() const { return feedforward_; }

  protected:
    Eigen::MatrixXd J_;
    Eigen::VectorXd error_;

  private:
    std::string name_;
    int dimension_;
    Eigen::MatrixXd K_;
    Eigen::VectorXd feedforward_;
};

/// Selected components of the pose of a frame. The six components are (x, y, z) of the position and
/// (x, y, z) of the orientation error log(R_d R^T) (eq. 4.10), all in the world frame; the task keeps the
/// rows listed in @p components, e.g. {0, 1} for a planar position, {0, 1, 2} for a position, {0, 1, 2, 5}
/// for position and yaw.
class FrameTask : public Task {
  public:
    FrameTask(std::string name, int frame, std::vector<int> components, const Eigen::Matrix4d& desired);

    void update(const RobotModel& model, const Eigen::VectorXd& q) override;
    void set_desired(const Eigen::Matrix4d& desired) { desired_ = desired; }
    const Eigen::Matrix4d& desired() const { return desired_; }
    int frame() const { return frame_; }
    const std::vector<int>& components() const { return components_; }

  private:
    int frame_;
    std::vector<int> components_;
    Eigen::Matrix4d desired_;
};

/// One joint (entry @p control_index of u) to a desired position. J is the unit row e_j^T.
class JointTask : public Task {
  public:
    JointTask(std::string name, int control_index, double desired);

    void update(const RobotModel& model, const Eigen::VectorXd& q) override;
    void set_desired(double desired) { desired_ = desired; }
    double desired() const { return desired_; }
    int control_index() const { return control_index_; }

  private:
    int control_index_;
    double desired_;
};

}  // namespace manipulator_control
