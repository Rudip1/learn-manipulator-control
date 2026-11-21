#include "manipulator_control/redundancy.hpp"

#include <Eigen/Cholesky>
#include <stdexcept>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/singularities.hpp"

namespace manipulator_control {

Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J, double tol) {
    const Eigen::Index n = J.cols();
    return Eigen::MatrixXd::Identity(n, n) - pseudoinverse(J, tol) * J;  // eq. (6.3)
}

Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J, const Eigen::MatrixXd& J_inverse) {
    if (J_inverse.rows() != J.cols() || J_inverse.cols() != J.rows())
        throw std::invalid_argument("null_space_projector: J_inverse must be n x m");
    const Eigen::Index n = J.cols();
    return Eigen::MatrixXd::Identity(n, n) - J_inverse * J;
}

Eigen::MatrixXd weighted_pseudoinverse(const Eigen::MatrixXd& J, const Eigen::MatrixXd& W, double tol) {
    if (W.rows() != J.cols() || W.cols() != J.cols())
        throw std::invalid_argument("weighted_pseudoinverse: W must be n x n");
    // Substitute qdot = L^-T y with W = L L^T: qdot^T W qdot = |y|^2 and J qdot = (J L^-T) y, so the
    // minimum-norm y is (J L^-T)^+ v. Eq. (6.6).
    const Eigen::LLT<Eigen::MatrixXd> llt(W);
    if (llt.info() != Eigen::Success)
        throw std::invalid_argument("weighted_pseudoinverse: W must be symmetric positive definite");
    const Eigen::MatrixXd L_inv_T = llt.matrixU().solve(Eigen::MatrixXd::Identity(W.rows(), W.cols()));
    return L_inv_T * pseudoinverse(J * L_inv_T, tol);
}

namespace {

void check_range(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max) {
    if (q_min.size() != q.size() || q_max.size() != q.size())
        throw std::invalid_argument("joint centering: q_min and q_max must have one entry per joint");
    if (((q_max - q_min).array() <= 0.0).any())
        throw std::invalid_argument("joint centering: q_max must be larger than q_min");
}

Eigen::VectorXd saturate(Eigen::VectorXd qdot, double max_speed) {
    const double peak = qdot.cwiseAbs().maxCoeff();
    if (peak > max_speed) qdot *= max_speed / peak;
    return qdot;
}

}  // namespace

double joint_centering_cost(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min,
                            const Eigen::VectorXd& q_max) {
    check_range(q, q_min, q_max);
    const Eigen::ArrayXd r = (q - 0.5 * (q_min + q_max)).array() / (q_max - q_min).array();
    return r.square().sum() / (2.0 * static_cast<double>(q.size()));  // eq. (6.9)
}

Eigen::VectorXd joint_centering_gradient(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min,
                                         const Eigen::VectorXd& q_max) {
    check_range(q, q_min, q_max);
    const Eigen::ArrayXd range = (q_max - q_min).array();
    const Eigen::ArrayXd g = (q - 0.5 * (q_min + q_max)).array() / range.square();
    return (g / static_cast<double>(q.size())).matrix();  // eq. (6.10)
}

double task_manipulability(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space) {
    return manipulability(task_jacobian(space, geometric_jacobian(chain, q)));
}

Eigen::VectorXd manipulability_gradient(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                        double h) {
    Eigen::VectorXd g(q.size());
    for (Eigen::Index i = 0; i < q.size(); ++i) {  // eq. (6.11)
        Eigen::VectorXd qp = q, qm = q;
        qp(i) += h;
        qm(i) -= h;
        g(i) = (task_manipulability(chain, qp, space) - task_manipulability(chain, qm, space)) / (2.0 * h);
    }
    return g;
}

double secondary_objective_value(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                 const RedundancyConfig& redundancy) {
    switch (redundancy.objective) {
        case SecondaryObjective::None:
            return 0.0;
        case SecondaryObjective::JointCentering:
            return joint_centering_cost(q, redundancy.q_min, redundancy.q_max);
        case SecondaryObjective::Manipulability:
            return task_manipulability(chain, q, space);
    }
    throw std::invalid_argument("secondary_objective_value: unknown objective");
}

Eigen::VectorXd secondary_velocity(const SerialChain& chain, const Eigen::VectorXd& q, TaskSpace space,
                                   const RedundancyConfig& redundancy) {
    switch (redundancy.objective) {
        case SecondaryObjective::None:
            return Eigen::VectorXd::Zero(q.size());
        case SecondaryObjective::JointCentering:  // descend H
            return -redundancy.k0 * joint_centering_gradient(q, redundancy.q_min, redundancy.q_max);
        case SecondaryObjective::Manipulability:  // ascend w
            return redundancy.k0 * manipulability_gradient(chain, q, space);
    }
    throw std::invalid_argument("secondary_velocity: unknown objective");
}

namespace {

Eigen::VectorXd gradient_projection(const SerialChain& chain, const Eigen::VectorXd& q,
                                    const Eigen::Matrix4d& T_desired, const Eigen::VectorXd& feedforward,
                                    const ResolvedRateConfig& config, const RedundancyConfig& redundancy,
                                    Eigen::VectorXd* error_out) {
    const Eigen::VectorXd e = task_error(config.space, T_desired, chain.end_effector(q));
    const Eigen::MatrixXd J = task_jacobian(config.space, geometric_jacobian(chain, q));
    Eigen::VectorXd v = config.gain * e;
    if (feedforward.size() == v.size()) v += feedforward;
    if (error_out != nullptr) *error_out = e;
    const double lambda =
        config.method == InverseMethod::DampedLeastSquares
            ? damping_factor(J, config.damping_schedule, config.damping, config.damping_threshold)
            : config.damping;
    const Eigen::MatrixXd J_inv = generalized_inverse(J, config.method, lambda);
    const Eigen::VectorXd z = secondary_velocity(chain, q, config.space, redundancy);
    const Eigen::VectorXd null_term = redundancy.project ? Eigen::VectorXd(null_space_projector(J, J_inv) * z) : z;
    return saturate(J_inv * v + null_term, config.max_joint_speed);  // eq. (6.8)
}

}  // namespace

Eigen::VectorXd redundancy_step(const SerialChain& chain, const Eigen::VectorXd& q, const Eigen::Matrix4d& T_desired,
                                const Eigen::VectorXd& feedforward, const ResolvedRateConfig& config,
                                const RedundancyConfig& redundancy) {
    return gradient_projection(chain, q, T_desired, feedforward, config, redundancy, nullptr);
}

RedundancyTrajectory simulate_redundancy(const SerialChain& chain, const Eigen::VectorXd& q0,
                                         const Eigen::Matrix4d& T_desired, const ResolvedRateConfig& config,
                                         const RedundancyConfig& redundancy) {
    const int n = chain.dof(), m = task_dimension(config.space), N = config.steps + 1;
    RedundancyTrajectory out;
    Trajectory& traj = out.trajectory;
    traj.t = Eigen::VectorXd::LinSpaced(N, 0.0, config.dt * config.steps);
    traj.q = Eigen::MatrixXd::Zero(N, n);
    traj.qdot = Eigen::MatrixXd::Zero(N, n);
    traj.ee_position = Eigen::MatrixXd::Zero(N, 3);
    traj.error = Eigen::MatrixXd::Zero(N, m);
    traj.error_norm = Eigen::VectorXd::Zero(N);
    out.objective = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd q = q0, e;
    const Eigen::VectorXd no_feedforward;
    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd qdot =
            gradient_projection(chain, q, T_desired, no_feedforward, config, redundancy, &e);
        traj.q.row(k) = q.transpose();
        traj.ee_position.row(k) = chain.end_effector(q).block<3, 1>(0, 3).transpose();
        traj.error.row(k) = e.transpose();
        traj.error_norm(k) = e.norm();
        out.objective(k) = secondary_objective_value(chain, q, config.space, redundancy);
        if (k == N - 1) break;
        traj.qdot.row(k) = qdot.transpose();
        q += config.dt * qdot;
    }
    return out;
}

}  // namespace manipulator_control
