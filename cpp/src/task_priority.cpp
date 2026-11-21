#include "manipulator_control/task_priority.hpp"

#include <stdexcept>

namespace manipulator_control {

namespace {

void check_level(const PriorityLevel& level, int n) {
    if (level.J.cols() != n || level.J.rows() != level.v.size())
        throw std::invalid_argument("task priority: each level needs an m x n Jacobian and an m-vector");
}

Eigen::VectorXd saturate(Eigen::VectorXd u, double max_speed) {
    const double peak = u.size() > 0 ? u.cwiseAbs().maxCoeff() : 0.0;
    if (peak > max_speed) u *= max_speed / peak;
    return u;
}

}  // namespace

Eigen::MatrixXd hierarchy_projector(const std::vector<PriorityLevel>& levels, int n, int count) {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n);
    for (int i = 0; i < count && i < static_cast<int>(levels.size()); ++i) {
        check_level(levels[size_t(i)], n);
        const Eigen::MatrixXd J_bar = levels[size_t(i)].J * P;  // augmented Jacobian, eq. (7.4)
        P -= pseudoinverse(J_bar) * J_bar;                     // eq. (7.6)
    }
    return P;
}

Eigen::VectorXd task_priority_velocity(const std::vector<PriorityLevel>& levels, int n,
                                       const PriorityConfig& config) {
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n);
    for (const PriorityLevel& level : levels) {
        check_level(level, n);
        const Eigen::MatrixXd J_bar = level.J * P;
        if (config.method == PriorityMethod::Recursive) {
            // Eq. (7.5): correct what the higher levels leave of this task, inside their null space.
            u += generalized_inverse(J_bar, config.inverse, config.damping) * (level.v - level.J * u);
        } else {
            // Eq. (7.10): invert the task alone, then project.
            u += P * generalized_inverse(level.J, config.inverse, config.damping) * level.v;
        }
        P -= pseudoinverse(J_bar) * J_bar;  // eq. (7.6), always with the exact pseudoinverse
    }
    return saturate(u, config.max_speed);
}

Eigen::VectorXd task_priority_step(const RobotModel& model, const Eigen::VectorXd& q, const TaskList& tasks,
                                   const PriorityConfig& config) {
    std::vector<PriorityLevel> levels;
    for (const auto& task : tasks) {
        task->update(model, q);
        if (task->is_active()) levels.push_back({task->jacobian(), task->velocity()});
    }
    return task_priority_velocity(levels, model.control_size(), config);
}

PriorityTrajectory simulate_task_priority(const RobotModel& model, const Eigen::VectorXd& q0,
                                          const TaskList& tasks, const PriorityConfig& config, double dt,
                                          int steps) {
    const int N = steps + 1, T = static_cast<int>(tasks.size());
    PriorityTrajectory traj;
    traj.t = Eigen::VectorXd::LinSpaced(N, 0.0, dt * steps);
    traj.q = Eigen::MatrixXd::Zero(N, model.state_size());
    traj.u = Eigen::MatrixXd::Zero(N, model.control_size());
    traj.error_norm = Eigen::MatrixXd::Zero(N, T);
    traj.active = Eigen::MatrixXd::Zero(N, T);
    traj.ee_position = Eigen::MatrixXd::Zero(N, 3);
    Eigen::VectorXd q = q0;
    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd u = task_priority_step(model, q, tasks, config);
        traj.q.row(k) = q.transpose();
        traj.ee_position.row(k) = model.frame_pose(q, -1).block<3, 1>(0, 3).transpose();
        for (int i = 0; i < T; ++i) {
            traj.error_norm(k, i) = tasks[size_t(i)]->error().norm();
            traj.active(k, i) = tasks[size_t(i)]->is_active() ? 1.0 : 0.0;
        }
        if (k == N - 1) break;
        traj.u.row(k) = u.transpose();
        q = model.integrate(q, u, dt);
    }
    return traj;
}

}  // namespace manipulator_control
