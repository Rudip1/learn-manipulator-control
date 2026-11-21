#pragma once
/// @file inequality_tasks.hpp
/// Set-based (inequality) tasks sigma_min <= sigma(q) <= sigma_max that enter the hierarchy only near a bound
/// (chapter 8, 1_theory/08_inequality_tasks.md).

#include <Eigen/Core>
#include <limits>
#include <string>

#include "manipulator_control/tasks.hpp"

namespace manipulator_control {

/// Activation state of a set-based task: 0 inactive, +1 guarding the lower bound, -1 guarding the upper bound
/// (the sign is the direction the task pushes sigma).
int next_activation(int state, double sigma, double sigma_min, double sigma_max, double alpha, double delta);

/// A scalar task sigma(q) kept inside [sigma_min, sigma_max]. It switches on when sigma comes within @p alpha
/// of a bound and off when sigma is @p delta away from it again (delta >= alpha; delta > alpha gives
/// hysteresis). While active its error is the activation, +1 or -1, so it asks for the task velocity
/// v = K * (+-1): a push away from the bound at a constant rate until the release threshold is crossed.
/// Eqs. (8.2)-(8.4).
class SetBasedTask : public Task {
  public:
    SetBasedTask(std::string name, double sigma_min, double sigma_max, double alpha, double delta);

    void update(const RobotModel& model, const Eigen::VectorXd& q) final;
    bool is_active() const override { return activation_ != 0; }
    int activation() const { return activation_; }
    void reset() { activation_ = 0; }
    double value() const { return sigma_; }
    double sigma_min() const { return sigma_min_; }
    double sigma_max() const { return sigma_max_; }
    double alpha() const { return alpha_; }
    double delta() const { return delta_; }

  protected:
    /// The task variable sigma(q) and its 1 x control_size Jacobian.
    virtual double compute(const RobotModel& model, const Eigen::VectorXd& q, Eigen::RowVectorXd& J) const = 0;

  private:
    double sigma_min_, sigma_max_, alpha_, delta_;
    int activation_ = 0;
    double sigma_ = 0.0;
};

/// Joint limit q_min <= q_j <= q_max for entry @p control_index of u. Eq. (8.5).
class JointLimitTask : public SetBasedTask {
  public:
    JointLimitTask(std::string name, int control_index, double q_min, double q_max, double alpha, double delta);
    int control_index() const { return control_index_; }

  protected:
    double compute(const RobotModel& model, const Eigen::VectorXd& q, Eigen::RowVectorXd& J) const override;

  private:
    int control_index_;
};

/// Distance from the origin of frame @p frame to a point obstacle at @p center, at least @p radius. With
/// @p planar true only (x, y) are used. sigma = |p - c|, J = (p - c)^T J_p / |p - c|. Eq. (8.6).
class ObstacleTask : public SetBasedTask {
  public:
    ObstacleTask(std::string name, int frame, const Eigen::Vector3d& center, double radius, double alpha,
                 double delta, bool planar = false);
    const Eigen::Vector3d& center() const { return center_; }
    void set_center(const Eigen::Vector3d& center) { center_ = center; }
    int frame() const { return frame_; }

  protected:
    double compute(const RobotModel& model, const Eigen::VectorXd& q, Eigen::RowVectorXd& J) const override;

  private:
    int frame_;
    Eigen::Vector3d center_;
    bool planar_;
};

}  // namespace manipulator_control
