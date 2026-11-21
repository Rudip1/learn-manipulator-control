// Chapter 6: redundancy and null space. References: a null-space basis computed independently by LU
// (FullPivLU::kernel), the KKT system of the weighted minimum-norm problem, finite differences, the closed
// form w = l1 l2 |sin q2|, and the end-effector staying put under self-motion.

#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/redundancy.hpp"
#include "manipulator_control/robots.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

namespace {

Eigen::MatrixXd random_matrix(test_helpers::Rng& rng, int rows, int cols) {
    Eigen::MatrixXd A(rows, cols);
    for (int i = 0; i < rows; ++i) A.row(i) = rng.vector(cols, -1, 1).transpose();
    return A;
}

}  // namespace

TEST_CASE("null-space projector is the orthogonal projector onto ker J", "[redundancy]") {
    test_helpers::Rng rng(6);
    for (int trial = 0; trial < 5; ++trial) {
        Eigen::MatrixXd J = random_matrix(rng, 3, 7);
        if (trial % 2 == 1) J.row(2) = 0.4 * J.row(0) - 1.3 * J.row(1);  // rank 2
        const Eigen::MatrixXd N = null_space_projector(J);
        CHECK(near(J * N, Eigen::MatrixXd::Zero(3, 7), 1e-12));
        CHECK(near(N * N, N, 1e-12));                        // idempotent
        CHECK(near(N, Eigen::MatrixXd(N.transpose()), 1e-12));  // symmetric
        // Independent reference: P = Z (Z^T Z)^-1 Z^T for any basis Z of the kernel (eq. 6.4).
        const Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
        const Eigen::MatrixXd Z = lu.kernel();
        CHECK(Z.cols() == 7 - lu.rank());
        const Eigen::MatrixXd P = Z * (Z.transpose() * Z).inverse() * Z.transpose();
        CHECK(near(N, P, 1e-10));
        CHECK(std::abs(N.trace() - double(7 - lu.rank())) < 1e-10);  // dimension of the self-motion
    }
}

TEST_CASE("weighted pseudoinverse solves the weighted minimum-norm problem", "[redundancy]") {
    // min qdot^T W qdot s.t. J qdot = v. KKT: [W J^T; J 0] [qdot; mu] = [0; v] (eq. 6.5).
    test_helpers::Rng rng(7);
    const int m = 2, n = 5;
    const Eigen::MatrixXd J = random_matrix(rng, m, n);
    const Eigen::MatrixXd A = random_matrix(rng, n, n);
    const Eigen::MatrixXd W = A * A.transpose() + 0.5 * Eigen::MatrixXd::Identity(n, n);
    const Eigen::VectorXd v = rng.vector(m, -1, 1);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n + m, n + m);
    K.topLeftCorner(n, n) = W;
    K.topRightCorner(n, m) = J.transpose();
    K.bottomLeftCorner(m, n) = J;
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + m);
    rhs.tail(m) = v;
    const Eigen::VectorXd kkt = K.fullPivLu().solve(rhs);
    const Eigen::MatrixXd JW = weighted_pseudoinverse(J, W);
    CHECK(near(JW * v, kkt.head(n), 1e-10));
    CHECK(near(JW, W.inverse() * J.transpose() * (J * W.inverse() * J.transpose()).inverse(), 1e-10));
    CHECK(near(weighted_pseudoinverse(J, Eigen::MatrixXd::Identity(n, n)), pseudoinverse(J), 1e-12));
    // A heavier weight on joint 0 makes it move less.
    Eigen::MatrixXd W0 = Eigen::MatrixXd::Identity(n, n);
    W0(0, 0) = 100.0;
    CHECK(std::abs((weighted_pseudoinverse(J, W0) * v)(0)) < std::abs((pseudoinverse(J) * v)(0)));
}

TEST_CASE("joint-centering gradient matches finite differences", "[redundancy]") {
    test_helpers::Rng rng(8);
    const Eigen::VectorXd q_min = Eigen::VectorXd::Constant(4, -2.0), q_max = Eigen::VectorXd::Constant(4, 1.0);
    for (int k = 0; k < 10; ++k) {
        const Eigen::VectorXd q = rng.vector(4, -2, 1);
        Eigen::VectorXd fd(4);
        for (int i = 0; i < 4; ++i) {
            Eigen::VectorXd qp = q, qm = q;
            qp(i) += 1e-6;
            qm(i) -= 1e-6;
            fd(i) = (joint_centering_cost(qp, q_min, q_max) - joint_centering_cost(qm, q_min, q_max)) / 2e-6;
        }
        CHECK(near(joint_centering_gradient(q, q_min, q_max), fd, 1e-8));
    }
    CHECK(joint_centering_cost(Eigen::VectorXd::Constant(4, -0.5), q_min, q_max) == 0.0);  // middle of range
}

TEST_CASE("manipulability gradient of the two-link arm is (0, l1 l2 cos q2 sign(sin q2))", "[redundancy]") {
    // Derivative of eq. (5.3).
    const double l1 = 0.75, l2 = 0.5;
    const SerialChain arm = robots::planar({l1, l2});
    for (double q2 : {0.3, 1.2, 2.5, -0.8}) {
        const Eigen::Vector2d q(0.4, q2);
        const Eigen::VectorXd g = manipulability_gradient(arm, q, TaskSpace::PlanarPosition);
        const double sign = q2 > 0 ? 1.0 : -1.0;
        CHECK(std::abs(g(0)) < 1e-8);
        CHECK(std::abs(g(1) - l1 * l2 * std::cos(q2) * sign) < 1e-8);
    }
}

TEST_CASE("self-motion moves the joints but not the end effector", "[redundancy]") {
    // Section 6.2: qdot = N z, integrated with a small step, keeps the end effector where it was.
    const SerialChain arm = robots::planar({0.5, 0.4, 0.3});
    Eigen::VectorXd q(3);
    q << 0.2, 0.9, -0.6;
    const Eigen::Vector3d p0 = arm.end_effector(q).block<3, 1>(0, 3);
    const Eigen::VectorXd q_start = q;
    const Eigen::Vector3d z(0.0, 0.0, 1.0);
    for (int k = 0; k < 1000; ++k) {
        const Eigen::MatrixXd J = geometric_jacobian(arm, q).topRows(2);
        q += 1e-3 * null_space_projector(J) * z;
    }
    CHECK((q - q_start).norm() > 0.5);
    CHECK((arm.end_effector(q).block<3, 1>(0, 3) - p0).norm() < 1e-3);  // drift of the Euler integration
}

TEST_CASE("gradient projection reaches the target and improves the secondary objective", "[redundancy]") {
    // Section 6.4: the same target with and without the null-space term.
    const SerialChain arm = robots::planar({0.5, 0.4, 0.3});
    Eigen::VectorXd q0(3), q_min(3), q_max(3);
    q0 << 1.2, -1.5, 1.6;
    q_min << -kPi / 2, -2.0, -2.0;
    q_max << kPi / 2, 2.0, 2.0;
    Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
    T_d.block<3, 1>(0, 3) << 0.6, 0.5, 0.0;
    ResolvedRateConfig config;
    config.space = TaskSpace::PlanarPosition;
    config.gain = 2.0;
    config.dt = 0.01;
    config.steps = 1000;
    RedundancyConfig none;
    none.q_min = q_min;
    none.q_max = q_max;
    RedundancyConfig centering = none;
    centering.objective = SecondaryObjective::JointCentering;
    centering.k0 = 20.0;
    const RedundancyTrajectory a = simulate_redundancy(arm, q0, T_d, config, none);
    const RedundancyTrajectory b = simulate_redundancy(arm, q0, T_d, config, centering);
    CHECK(a.trajectory.error_norm(config.steps) < 1e-6);
    // While the null-space motion is still going on, the Euler step leaks a little of it into the task
    // (section 6.4): a small residual error, not zero.
    CHECK(b.trajectory.error_norm(config.steps) < 1e-4);
    // Joint-centering cost of the final configurations (objective None records 0, so compute H here).
    const double H_a = joint_centering_cost(a.trajectory.q.bottomRows(1).transpose(), q_min, q_max);
    const double H_b = b.objective(config.steps);
    CHECK(H_b < 0.3 * H_a);  // 0.027 vs 0.098
    // Without the projector the secondary motion fights the task: error is no longer driven to zero.
    RedundancyConfig unprojected = centering;
    unprojected.project = false;
    const RedundancyTrajectory c = simulate_redundancy(arm, q0, T_d, config, unprojected);
    CHECK(c.trajectory.error_norm(config.steps) > 1e-3);
}

TEST_CASE("manipulability objective raises w of a redundant planar arm", "[redundancy]") {
    // n = 4 joints, m = 2 position rows: two degrees of redundancy. (The PUMA 560 would not do: its position
    // Jacobian does not depend on the wrist joints, which are exactly its null-space joints.)
    const SerialChain arm = robots::planar({0.4, 0.35, 0.3, 0.25});
    Eigen::VectorXd q0(4);
    q0 << 0.0, 2.0, -2.0, 2.0;
    Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
    T_d.block<3, 1>(0, 3) << 0.5, 0.3, 0.0;
    ResolvedRateConfig config;
    config.space = TaskSpace::PlanarPosition;
    config.gain = 2.0;
    config.dt = 0.01;
    config.steps = 1000;
    RedundancyConfig none, manip;
    manip.objective = SecondaryObjective::Manipulability;
    manip.k0 = 20.0;
    const RedundancyTrajectory a = simulate_redundancy(arm, q0, T_d, config, none);
    const RedundancyTrajectory b = simulate_redundancy(arm, q0, T_d, config, manip);
    const double w_a = task_manipulability(arm, a.trajectory.q.bottomRows(1).transpose(), config.space);
    CHECK(a.trajectory.error_norm(config.steps) < 1e-6);
    CHECK(b.trajectory.error_norm(config.steps) < 1e-6);
    CHECK(b.objective(config.steps) > 1.6 * w_a);  // 0.460 vs 0.265
}

TEST_CASE("the DLS projector leaks into the task by at most lambda / 2", "[redundancy]") {
    // Eq. (6.12): J (I - J_lambda^+ J) = sum_i lambda^2 s_i / (s_i^2 + lambda^2) u_i v_i^T, largest at s = lambda.
    const SerialChain arm = robots::planar({0.5, 0.4, 0.3});
    for (double lambda : {0.01, 0.05, 0.1}) {
        double worst = 0.0;
        for (double q2 = -0.5; q2 <= 0.5; q2 += 0.001) {
            const Eigen::MatrixXd J = geometric_jacobian(arm, Eigen::Vector3d(0.3, q2, 0.4)).topRows(2);
            const Eigen::MatrixXd leak = J * null_space_projector(J, dls_inverse(J, lambda));
            const double norm = leak.jacobiSvd().singularValues()(0);
            CHECK(norm <= lambda / 2 + 1e-12);
            worst = std::max(worst, norm);
        }
        CHECK(worst > 0.0);
        // The bound is reached when a singular value equals lambda.
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 3);
        D(0, 0) = 1.0;
        D(1, 1) = lambda;
        const Eigen::MatrixXd leak_D = D * null_space_projector(D, dls_inverse(D, lambda));
        CHECK(std::abs(leak_D.jacobiSvd().singularValues()(0) - lambda / 2) < 1e-12);
        const Eigen::MatrixXd J = geometric_jacobian(arm, Eigen::Vector3d(0.3, 0.9, 0.4)).topRows(2);
        CHECK(near(J * null_space_projector(J), Eigen::MatrixXd::Zero(2, 3), 1e-12));  // exact projector: none
    }
}
