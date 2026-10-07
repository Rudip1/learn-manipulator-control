#include "manipulator_control/resolved_rate.hpp"

#include <Eigen/SVD>
#include <stdexcept>

#include "manipulator_control/jacobian.hpp"

namespace manipulator_control {

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& J, double tol) {
    // Eq. (4.4): J^+ = V S^+ U^T, inverting only the singular values that are not negligible.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& s = svd.singularValues();
    const double cutoff = s.size() > 0 ? tol * s(0) : 0.0;
    Eigen::VectorXd s_inv = Eigen::VectorXd::Zero(s.size());
    for (Eigen::Index i = 0; i < s.size(); ++i) {
        if (s(i) > cutoff && s(i) > 0.0) s_inv(i) = 1.0 / s(i);
    }
    return svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();
}

Eigen::MatrixXd dls_inverse(const Eigen::MatrixXd& J, double lambda) {
    // Eq. (4.6). J J^T + lambda^2 I is symmetric positive definite for lambda > 0, so LDLT is safe.
    const Eigen::MatrixXd A =
        J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J.transpose() * A.ldlt().solve(Eigen::MatrixXd::Identity(J.rows(), J.rows()));
}

Eigen::MatrixXd generalized_inverse(const Eigen::MatrixXd& J, InverseMethod method, double lambda) {
    switch (method) {
        case InverseMethod::Transpose:
            return J.transpose();
        case InverseMethod::Pseudoinverse:
            return pseudoinverse(J);
        case InverseMethod::DampedLeastSquares:
            return dls_inverse(J, lambda);
    }
    throw std::invalid_argument("generalized_inverse: unknown method");
}

int task_dimension(TaskSpace space) {
    switch (space) {
        case TaskSpace::PlanarPosition:
            return 2;
        case TaskSpace::Position:
            return 3;
        case TaskSpace::Pose:
            return 6;
    }
    throw std::invalid_argument("task_dimension: unknown task space");
}

Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R) {
    return log_so3(R_desired * R.transpose());  // eq. (4.10)
}

Eigen::VectorXd task_error(TaskSpace space, const Eigen::Matrix4d& T_desired, const Eigen::Matrix4d& T) {
    const Eigen::Vector3d ep = T_desired.block<3, 1>(0, 3) - T.block<3, 1>(0, 3);
    switch (space) {
        case TaskSpace::PlanarPosition:
            return ep.head<2>();
        case TaskSpace::Position:
            return ep;
        case TaskSpace::Pose: {
            Eigen::VectorXd e(6);
            e << ep, orientation_error(T_desired.topLeftCorner<3, 3>(), T.topLeftCorner<3, 3>());
            return e;
        }
    }
    throw std::invalid_argument("task_error: unknown task space");
}

Eigen::MatrixXd task_jacobian(TaskSpace space, const Eigen::MatrixXd& J) {
    return space == TaskSpace::Pose ? J : Eigen::MatrixXd(J.topRows(task_dimension(space)));
}

namespace {

Eigen::VectorXd saturate(Eigen::VectorXd qdot, double max_speed) {
    const double peak = qdot.cwiseAbs().maxCoeff();
    if (peak > max_speed) qdot *= max_speed / peak;  // uniform scaling keeps the direction
    return qdot;
}

Eigen::VectorXd control_law(const SerialChain& chain, const Eigen::VectorXd& q,
                            const Eigen::Matrix4d& T_desired, const Eigen::VectorXd& feedforward,
                            const ResolvedRateConfig& config, Eigen::VectorXd* error_out) {
    const Eigen::Matrix4d T = chain.end_effector(q);
    const Eigen::VectorXd e = task_error(config.space, T_desired, T);
    const Eigen::MatrixXd J = task_jacobian(config.space, geometric_jacobian(chain, q));
    Eigen::VectorXd v = config.gain * e;
    if (feedforward.size() == v.size()) v += feedforward;
    if (error_out != nullptr) *error_out = e;
    const double lambda =
        config.method == InverseMethod::DampedLeastSquares
            ? damping_factor(J, config.damping_schedule, config.damping, config.damping_threshold)
            : config.damping;
    return saturate(generalized_inverse(J, config.method, lambda) * v, config.max_joint_speed);  // eq. (4.3)
}

Trajectory allocate(int steps, int n, int m, double dt) {
    Trajectory traj;
    const int N = steps + 1;
    traj.t = Eigen::VectorXd::LinSpaced(N, 0.0, dt * steps);
    traj.q = Eigen::MatrixXd::Zero(N, n);
    traj.qdot = Eigen::MatrixXd::Zero(N, n);
    traj.ee_position = Eigen::MatrixXd::Zero(N, 3);
    traj.error = Eigen::MatrixXd::Zero(N, m);
    traj.error_norm = Eigen::VectorXd::Zero(N);
    return traj;
}

}  // namespace

Eigen::VectorXd resolved_rate_step(const SerialChain& chain, const Eigen::VectorXd& q,
                                   const Eigen::Matrix4d& T_desired, const Eigen::VectorXd& feedforward,
                                   const ResolvedRateConfig& config) {
    return control_law(chain, q, T_desired, feedforward, config, nullptr);
}

Trajectory simulate_resolved_rate(const SerialChain& chain, const Eigen::VectorXd& q0,
                                  const Eigen::Matrix4d& T_desired, const ResolvedRateConfig& config) {
    const int m = task_dimension(config.space);
    Trajectory traj = allocate(config.steps, chain.dof(), m, config.dt);
    Eigen::VectorXd q = q0, e;
    const Eigen::VectorXd no_feedforward;
    for (int k = 0; k <= config.steps; ++k) {
        const Eigen::VectorXd qdot = control_law(chain, q, T_desired, no_feedforward, config, &e);
        traj.q.row(k) = q.transpose();
        traj.ee_position.row(k) = chain.end_effector(q).block<3, 1>(0, 3).transpose();
        traj.error.row(k) = e.transpose();
        traj.error_norm(k) = e.norm();
        if (k == config.steps) break;
        traj.qdot.row(k) = qdot.transpose();
        q += config.dt * qdot;  // explicit Euler, section 4.6
    }
    return traj;
}

Trajectory simulate_position_tracking(const SerialChain& chain, const Eigen::VectorXd& q0,
                                      const Eigen::MatrixXd& positions, const Eigen::MatrixXd& velocities,
                                      const ResolvedRateConfig& config_in) {
    ResolvedRateConfig config = config_in;
    const Eigen::Index m = positions.cols();
    if (m != 2 && m != 3)
        throw std::invalid_argument("simulate_position_tracking: positions must have 2 or 3 columns");
    if (velocities.rows() != positions.rows() || velocities.cols() != m)
        throw std::invalid_argument("simulate_position_tracking: velocities must match positions");
    config.space = m == 2 ? TaskSpace::PlanarPosition : TaskSpace::Position;
    config.steps = static_cast<int>(positions.rows()) - 1;
    Trajectory traj = allocate(config.steps, chain.dof(), static_cast<int>(m), config.dt);
    Eigen::VectorXd q = q0, e;
    for (int k = 0; k <= config.steps; ++k) {
        Eigen::Matrix4d T_desired = Eigen::Matrix4d::Identity();
        T_desired.block(0, 3, m, 1) = positions.row(k).transpose();
        const Eigen::VectorXd ff = velocities.row(k).transpose();
        const Eigen::VectorXd qdot = control_law(chain, q, T_desired, ff, config, &e);
        traj.q.row(k) = q.transpose();
        traj.ee_position.row(k) = chain.end_effector(q).block<3, 1>(0, 3).transpose();
        traj.error.row(k) = e.transpose();
        traj.error_norm(k) = e.norm();
        if (k == config.steps) break;
        traj.qdot.row(k) = qdot.transpose();
        q += config.dt * qdot;
    }
    return traj;
}

Trajectory simulate_pose_tracking(const SerialChain& chain, const Eigen::VectorXd& q0,
                                  const std::vector<Eigen::Matrix4d>& poses, const Eigen::MatrixXd& twists,
                                  const ResolvedRateConfig& config_in) {
    if (poses.empty() || twists.rows() != static_cast<Eigen::Index>(poses.size()) || twists.cols() != 6)
        throw std::invalid_argument("simulate_pose_tracking: need one 6-vector twist per pose");
    ResolvedRateConfig config = config_in;
    config.space = TaskSpace::Pose;
    config.steps = static_cast<int>(poses.size()) - 1;
    Trajectory traj = allocate(config.steps, chain.dof(), 6, config.dt);
    Eigen::VectorXd q = q0, e;
    for (int k = 0; k <= config.steps; ++k) {
        const Eigen::VectorXd ff = twists.row(k).transpose();
        const Eigen::VectorXd qdot = control_law(chain, q, poses[size_t(k)], ff, config, &e);
        traj.q.row(k) = q.transpose();
        traj.ee_position.row(k) = chain.end_effector(q).block<3, 1>(0, 3).transpose();
        traj.error.row(k) = e.transpose();
        traj.error_norm(k) = e.norm();
        if (k == config.steps) break;
        traj.qdot.row(k) = qdot.transpose();
        q += config.dt * qdot;
    }
    return traj;
}

}  // namespace manipulator_control
