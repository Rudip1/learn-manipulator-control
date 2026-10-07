// Chapter 4: generalised inverses and resolved-rate control. References: the Penrose conditions, the SVD form
// of the DLS inverse, and the closed-loop error dynamics derived in 1_theory/04_resolved_rate_control.md.

#include <Eigen/SVD>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/resolved_rate.hpp"
#include "manipulator_control/robots.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

namespace {
Eigen::Matrix4d position_target(double x, double y, double z = 0.0) { return translation({x, y, z}); }
}  // namespace

TEST_CASE("pseudoinverse satisfies the four Penrose conditions", "[resolved_rate]") {
    test_helpers::Rng rng;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 6);
    for (int i = 0; i < 4; ++i) A.row(i) = rng.vector(6, -1, 1).transpose();
    A.row(3) = A.row(0) + 2.0 * A.row(1);  // rank 3
    const Eigen::MatrixXd P = pseudoinverse(A);
    CHECK(near(A * P * A, A, 1e-12));
    CHECK(near(P * A * P, P, 1e-12));
    CHECK(near(Eigen::MatrixXd((A * P).transpose()), A * P, 1e-12));
    CHECK(near(Eigen::MatrixXd((P * A).transpose()), P * A, 1e-12));
}

TEST_CASE("pseudoinverse reduces to the right inverse and to the inverse", "[resolved_rate]") {
    test_helpers::Rng rng(2);
    Eigen::MatrixXd J(2, 3);
    J << rng.vector(3, -1, 1).transpose(), rng.vector(3, -1, 1).transpose();
    CHECK(near(pseudoinverse(J), Eigen::MatrixXd(J.transpose() * (J * J.transpose()).inverse()), 1e-12));
    Eigen::Matrix3d A;
    A << 2, 1, 0, 0, 1, 1, 1, 0, 3;
    CHECK(near(pseudoinverse(A), Eigen::MatrixXd(A.inverse()), 1e-12));
}

TEST_CASE("DLS inverse equals the filtered SVD form", "[resolved_rate]") {
    // Eqs. (4.6) and (5.5): J^T (J J^T + l^2 I)^-1 = sum_i s_i / (s_i^2 + l^2) v_i u_i^T.
    test_helpers::Rng rng(3);
    Eigen::MatrixXd J(3, 5);
    for (int i = 0; i < 3; ++i) J.row(i) = rng.vector(5, -1, 1).transpose();
    const double lambda = 0.3;
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd s = svd.singularValues();
    const Eigen::VectorXd f = s.array() / (s.array().square() + lambda * lambda);
    CHECK(near(dls_inverse(J, lambda),
               Eigen::MatrixXd(svd.matrixV() * f.asDiagonal() * svd.matrixU().transpose()), 1e-12));
    CHECK(near(dls_inverse(J, 0.0), pseudoinverse(J),
               1e-10));  // full row rank: lambda = 0 is the pseudoinverse
}

TEST_CASE("orientation error is the rotation that takes R to R_d", "[resolved_rate]") {
    const Eigen::Matrix3d R = rotation_from_rpy(Eigen::Vector3d(0.1, 0.4, -0.3));
    const Eigen::Vector3d e_true(0.2, -0.1, 0.3);
    const Eigen::Matrix3d Rd = exp_so3(e_true) * R;
    CHECK(near(orientation_error(Rd, R), e_true, 1e-12));
    CHECK(near(orientation_error(R, R), Eigen::Vector3d::Zero(), 1e-12));
}

TEST_CASE("pseudoinverse control gives exponential convergence with rate K", "[resolved_rate]") {
    // Eq. (4.5): with full row rank, e(t) = exp(-K t) e(0); the Euler error at dt = 1e-4 is about K dt / 2.
    const SerialChain arm = robots::planar({0.75, 0.5});
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.gain = 2.0;
    cfg.dt = 1e-4;
    cfg.steps = 10000;  // 1 s
    const Trajectory traj =
        simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), position_target(0.0, 1.0), cfg);
    const double ratio = traj.error_norm(cfg.steps) / traj.error_norm(0);
    CHECK(std::abs(ratio / std::exp(-cfg.gain * 1.0) - 1.0) < 1e-3);
    // The direction of the error does not change (straight-line motion of the end-effector).
    const Eigen::Vector2d e0 = traj.error.row(0).transpose(), e1 = traj.error.row(5000).transpose();
    CHECK(std::abs(e0.normalized().dot(e1.normalized()) - 1.0) < 1e-6);
}

TEST_CASE("discrete-time gain limit: K dt below 2 converges and above 2 does not", "[resolved_rate]") {
    // Eq. (4.9): near the goal e_{k+1} = (1 - K dt) e_k.
    const SerialChain arm = robots::planar({0.75, 0.5});
    const Eigen::Vector2d q0(0.2, 0.5);
    const Eigen::Matrix4d T0 = arm.end_effector(q0);
    const Eigen::Matrix4d goal =
        position_target(T0(0, 3) + 1e-4, T0(1, 3) - 1e-4);  // tiny step: linear regime
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.dt = 0.01;
    cfg.steps = 1;
    for (double kdt : {0.5, 1.0, 1.5, 2.5}) {
        cfg.gain = kdt / cfg.dt;
        const Trajectory traj = simulate_resolved_rate(arm, q0, goal, cfg);
        CHECK(std::abs(traj.error_norm(1) / traj.error_norm(0) - std::abs(1.0 - kdt)) < 1e-3);
    }
}

TEST_CASE("Jacobian-transpose control decreases the error monotonically", "[resolved_rate]") {
    // Eq. (4.7): V = e^T e / 2 decreases while J^T e != 0, for a gain inside the discrete limit.
    const SerialChain arm = robots::planar({0.75, 0.5});
    ResolvedRateConfig cfg;
    cfg.method = InverseMethod::Transpose;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.gain = 1.0;
    cfg.dt = 1.0 / 60.0;
    cfg.steps = 1200;
    const Trajectory traj =
        simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), position_target(0.0, 1.0), cfg);
    for (int k = 1; k <= cfg.steps; ++k) CHECK(traj.error_norm(k) <= traj.error_norm(k - 1) + 1e-15);
    // Converges, but slowly: the rate along each singular direction is K s_i^2 (section 4.4).
    CHECK(traj.error_norm(cfg.steps) < 0.05 * traj.error_norm(0));
    cfg.method = InverseMethod::Pseudoinverse;
    const Trajectory pinv =
        simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), position_target(0.0, 1.0), cfg);
    CHECK(pinv.error_norm(cfg.steps) < 1e-6);
}

TEST_CASE("full pose control of the PUMA 560 converges", "[resolved_rate]") {
    const SerialChain arm = robots::puma560();
    Eigen::VectorXd q0(6), qg(6);
    q0 << 0.1, 0.6, -0.4, 0.3, 0.8, 0.2;
    qg << 0.5, 0.3, -0.1, -0.2, 1.1, -0.4;
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::Pose;
    cfg.gain = 2.0;
    cfg.dt = 0.01;
    cfg.steps = 800;
    const Trajectory traj = simulate_resolved_rate(arm, q0, arm.end_effector(qg), cfg);
    CHECK(traj.error_norm(cfg.steps) < 1e-6);
}

TEST_CASE("tracking with feedforward has no lag and without it a lag of v / K", "[resolved_rate]") {
    // Eq. (4.8): constant reference velocity v gives the steady error v / K without feedforward.
    const SerialChain arm = robots::planar({0.75, 0.5});
    const Eigen::Vector2d q0(0.3, 1.2);
    const Eigen::Vector2d p0 = arm.end_effector(q0).block<2, 1>(0, 3);
    const Eigen::Vector2d v(0.05, -0.02);
    const double dt = 0.002, K = 4.0;
    const int steps = 2000;  // 4 s
    Eigen::MatrixXd pos(steps + 1, 2), vel(steps + 1, 2), zero = Eigen::MatrixXd::Zero(steps + 1, 2);
    for (int k = 0; k <= steps; ++k) {
        pos.row(k) = (p0 + v * (k * dt)).transpose();
        vel.row(k) = v.transpose();
    }
    ResolvedRateConfig cfg;
    cfg.gain = K;
    cfg.dt = dt;
    const Trajectory with_ff = simulate_position_tracking(arm, q0, pos, vel, cfg);
    const Trajectory without_ff = simulate_position_tracking(arm, q0, pos, zero, cfg);
    CHECK(with_ff.error_norm.maxCoeff() < 1e-4);  // only the O(dt) Euler error remains
    CHECK(near(Eigen::Vector2d(without_ff.error.row(steps).transpose()), Eigen::Vector2d(v / K), 2e-4));
}

TEST_CASE("joint speed saturation scales the command uniformly", "[resolved_rate]") {
    const SerialChain arm = robots::planar({0.75, 0.5});
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.gain = 50.0;
    const Eigen::Vector2d q(0.2, 0.5);
    const Eigen::Matrix4d goal = position_target(0.0, 1.0);
    const Eigen::VectorXd free = resolved_rate_step(arm, q, goal, Eigen::VectorXd(), cfg);
    cfg.max_joint_speed = 0.5;
    const Eigen::VectorXd sat = resolved_rate_step(arm, q, goal, Eigen::VectorXd(), cfg);
    CHECK(std::abs(sat.cwiseAbs().maxCoeff() - 0.5) < 1e-12);
    CHECK(near(sat.normalized(), free.normalized(), 1e-12));
}

TEST_CASE("pose error including the axis-angle part decays as exp(-K t)", "[resolved_rate]") {
    // Section 4.7: with e_O = log(R_d R^T) and pseudoinverse control, de/dt = -K e exactly.
    const SerialChain arm = robots::puma560();
    Eigen::VectorXd q0(6), qg(6);
    q0 << 0.1, 0.6, -0.4, 0.3, 0.8, 0.2;
    qg << 0.4, 0.4, -0.2, -0.1, 1.0, -0.3;
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::Pose;
    cfg.gain = 2.0;
    cfg.dt = 1e-4;
    cfg.steps = 10000;
    const Trajectory traj = simulate_resolved_rate(arm, q0, arm.end_effector(qg), cfg);
    CHECK(std::abs(traj.error_norm(cfg.steps) / traj.error_norm(0) / std::exp(-2.0) - 1.0) < 1e-3);
    const Eigen::VectorXd e0 = traj.error.row(0), e1 = traj.error.row(cfg.steps);
    CHECK(std::abs(e0.normalized().dot(e1.normalized()) - 1.0) < 1e-6);  // the error keeps its direction
}
