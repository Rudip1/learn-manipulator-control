#pragma once
/// @file task_priority.hpp
/// Task-priority redundancy resolution for N prioritised tasks (chapter 7, 1_theory/07_task_priority.md).

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <vector>

#include "manipulator_control/resolved_rate.hpp"
#include "manipulator_control/robot_model.hpp"
#include "manipulator_control/tasks.hpp"

namespace manipulator_control {

/// How the levels of the hierarchy are combined.
enum class PriorityMethod {
    /// qdot_i = qdot_{i-1} + (J_i P_{i-1})^# (v_i - J_i qdot_{i-1}): each task is met as well as possible in
    /// the null space of all higher ones. Exact for compatible tasks; suffers algorithmic singularities.
    /// Eqs. (7.5)-(7.7).
    Recursive,
    /// qdot = sum_i P_{i-1} J_i^# v_i (Chiaverini): no algorithmic singularities, but lower tasks are only
    /// approximated even when they are compatible. Eq. (7.10).
    SingularityRobust,
};

/// Settings of the task-priority solver.
struct PriorityConfig {
    PriorityMethod method = PriorityMethod::Recursive;
    InverseMethod inverse = InverseMethod::Pseudoinverse;  ///< inverse used for each level (J^+ or DLS)
    double damping = 0.05;                                 ///< lambda of the DLS inverse
    /// Per-entry speed limit on u; the command is scaled down uniformly to respect it.
    double max_speed = std::numeric_limits<double>::infinity();
    /// Diagonal of the weight W on u (empty: W = I). Every level then minimises u^T W u instead of |u|^2, so a
    /// large weight makes that entry move less (chapter 9, eq. 9.8).
    Eigen::VectorXd weights;
};

/// One level of the hierarchy: Jacobian J_i (m_i x n) and desired task velocity v_i.
struct PriorityLevel {
    Eigen::MatrixXd J;
    Eigen::VectorXd v;
};

/// Velocity u (size n) for levels ordered from highest to lowest priority. Eqs. (7.5)-(7.7) or (7.10).
Eigen::VectorXd task_priority_velocity(const std::vector<PriorityLevel>& levels, int n,
                                       const PriorityConfig& config);

/// Projector onto the common null space of the first @p count levels: P_count of eq. (7.6).
Eigen::MatrixXd hierarchy_projector(const std::vector<PriorityLevel>& levels, int n, int count);

using TaskList = std::vector<std::shared_ptr<Task>>;

/// Update every task at q and return the command for the active ones, in list order. Section 7.6.
Eigen::VectorXd task_priority_step(const RobotModel& model, const Eigen::VectorXd& q, const TaskList& tasks,
                                   const PriorityConfig& config);

/// Logged result of simulate_task_priority with N = steps + 1 samples and T tasks.
struct PriorityTrajectory {
    Eigen::VectorXd t;           ///< N times [s]
    Eigen::MatrixXd q;           ///< N x state_size configurations
    Eigen::MatrixXd u;           ///< N x control_size commands (last row zero)
    Eigen::MatrixXd error_norm;  ///< N x T norms of each task's error
    Eigen::MatrixXd active;      ///< N x T, 1 where the task was active
    Eigen::MatrixXd ee_position; ///< N x 3 end-effector positions
};

/// Simulate the closed loop q_{k+1} = model.integrate(q_k, u_k, dt). The tasks keep their desired values;
/// change them between calls to follow a sequence of goals.
PriorityTrajectory simulate_task_priority(const RobotModel& model, const Eigen::VectorXd& q0,
                                          const TaskList& tasks, const PriorityConfig& config, double dt,
                                          int steps);

}  // namespace manipulator_control
