// Chapter 5: singularities. References: the closed-form manipulability of the planar arm and the PUMA wrist
// singularity from 1_theory/05_singularities.md, determinant and SVD identities, and the DLS joint-rate
// bound.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/resolved_rate.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/singularities.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

TEST_CASE("planar two-link manipulability is l1 l2 |sin q2|", "[singularities]") {
    // Eq. (5.3).
    const double l1 = 0.75, l2 = 0.5;
    const SerialChain arm = robots::planar({l1, l2});
    test_helpers::Rng rng;
    for (int k = 0; k < 20; ++k) {
        const Eigen::Vector2d q = rng.vector(2, -kPi, kPi);
        const Eigen::MatrixXd J = geometric_jacobian(arm, q).topRows(2);
        CHECK(std::abs(manipulability(J) - l1 * l2 * std::abs(std::sin(q(1)))) < 1e-14);
    }
    CHECK(manipulability(geometric_jacobian(arm, Eigen::Vector2d(0.4, 0.0)).topRows(2)) < 1e-15);
}

TEST_CASE("manipulability is sqrt(det(J J^T)) and the product of singular values", "[singularities]") {
    test_helpers::Rng rng(2);
    Eigen::MatrixXd J(3, 5);
    for (int i = 0; i < 3; ++i) J.row(i) = rng.vector(5, -1, 1).transpose();
    CHECK(std::abs(manipulability(J) - std::sqrt((J * J.transpose()).determinant())) < 1e-12);
    CHECK(manipulability(Eigen::MatrixXd(J.transpose())) == 0.0);  // more rows than columns
    const Eigen::VectorXd s = singular_values(J);
    CHECK(std::abs(condition_number(J) - s(0) / s(2)) < 1e-12);
}

TEST_CASE("PUMA 560 loses rank at the wrist singularity q5 = 0", "[singularities]") {
    // Section 5.1: with q5 = 0 the axes of joints 4 and 6 line up.
    const SerialChain arm = robots::puma560();
    Eigen::VectorXd q(6);
    q << 0.3, 0.5, -0.2, 0.7, 0.0, -0.4;
    CHECK(manipulability(geometric_jacobian(arm, q)) < 1e-12);
    CHECK(singular_values(geometric_jacobian(arm, q))(5) < 1e-12);
    q(4) = 0.6;
    CHECK(manipulability(geometric_jacobian(arm, q)) > 1e-3);
}

TEST_CASE("velocity ellipsoid axes are s_i u_i", "[singularities]") {
    const SerialChain arm = robots::planar({0.75, 0.5, 0.5});
    const Eigen::MatrixXd J = geometric_jacobian(arm, Eigen::Vector3d(0.2, 0.7, -0.4)).topRows(2);
    const Ellipsoid e = manipulability_ellipsoid(J);
    const Eigen::MatrixXd Jp = pseudoinverse(J);
    for (int i = 0; i < 2; ++i) {
        // The unit joint velocity along v_i = J^+ u_i s_i maps to s_i u_i.
        const Eigen::VectorXd qdot = Jp * e.axes.col(i) * e.radii(i);
        CHECK(std::abs(qdot.norm() - 1.0) < 1e-12);
        CHECK(near(J * qdot, e.axes.col(i) * e.radii(i), 1e-12));
    }
}

TEST_CASE("DLS solves the damped normal equations and bounds the joint rates", "[singularities]") {
    // Eq. (5.4): (J^T J + l^2 I) qdot = J^T v; eq. (5.6): |qdot| <= |v| / (2 l).
    test_helpers::Rng rng(4);
    const double lambda = 0.05;
    for (int k = 0; k < 50; ++k) {
        Eigen::MatrixXd J(2, 3);
        J << rng.vector(3, -1, 1).transpose(), rng.vector(3, -1, 1).transpose();
        J.row(1) = J.row(0) * rng.uniform(-1, 1) + 1e-4 * J.row(1);  // nearly singular
        const Eigen::VectorXd v = rng.vector(2, -1, 1);
        const Eigen::VectorXd qdot = dls_inverse(J, lambda) * v;
        const Eigen::MatrixXd normal = J.transpose() * J + lambda * lambda * Eigen::MatrixXd::Identity(3, 3);
        CHECK(near(normal * qdot, Eigen::VectorXd(J.transpose() * v), 1e-12));
        CHECK(qdot.norm() <= v.norm() / (2.0 * lambda) + 1e-12);
    }
    // The bound is attained when a singular value equals lambda.
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, 2);
    J(0, 0) = lambda;
    CHECK(std::abs((dls_inverse(J, lambda) * Eigen::VectorXd::Ones(1)).norm() - 1.0 / (2.0 * lambda)) <
          1e-12);
}

TEST_CASE("damping schedules vanish away from singularities and are continuous", "[singularities]") {
    const SerialChain arm = robots::planar({0.75, 0.5});
    auto J_at = [&](double q2) {
        return Eigen::MatrixXd(geometric_jacobian(arm, Eigen::Vector2d(0.3, q2)).topRows(2));
    };
    const double lmax = 0.2, w0 = 0.1, eps = 0.15;
    CHECK(damping_factor(J_at(1.2), DampingSchedule::Constant, lmax, w0) == lmax);
    // Far from the singularity both variable schedules give zero damping, i.e. the pseudoinverse.
    CHECK(damping_factor(J_at(1.2), DampingSchedule::Manipulability, lmax, w0) == 0.0);
    CHECK(damping_factor(J_at(1.2), DampingSchedule::MinSingularValue, lmax, eps) == 0.0);
    // At the singularity both reach lambda_max.
    CHECK(std::abs(damping_factor(J_at(0.0), DampingSchedule::Manipulability, lmax, w0) - lmax) < 1e-12);
    CHECK(std::abs(damping_factor(J_at(0.0), DampingSchedule::MinSingularValue, lmax, eps) - lmax) < 1e-12);
    // Continuity at the threshold: w = l1 l2 |sin q2| = w0 at q2 = asin(w0 / (l1 l2)).
    const double q2_star = std::asin(w0 / (0.75 * 0.5));
    CHECK(damping_factor(J_at(q2_star - 1e-6), DampingSchedule::Manipulability, lmax, w0) < 1e-8);
}

TEST_CASE("an unreachable target: DLS keeps joint rates bounded, the pseudoinverse does not",
          "[singularities]") {
    const SerialChain arm = robots::planar({0.75, 0.5});
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.dt = 1.0 / 60.0;
    cfg.steps = 600;
    const Eigen::Matrix4d far = translation({1.5, 0.0, 0.0});
    cfg.method = InverseMethod::DampedLeastSquares;
    cfg.damping = 0.1;
    const Trajectory dls = simulate_resolved_rate(arm, Eigen::Vector2d(0.3, 0.6), far, cfg);
    const double bound = cfg.gain * dls.error_norm.maxCoeff() / (2.0 * cfg.damping);  // eq. (5.6)
    CHECK(dls.qdot.cwiseAbs().maxCoeff() <= bound);
    CHECK(std::abs(dls.error_norm(cfg.steps) - 0.25) < 1e-3);  // stretched out: 1.5 - (l1 + l2)
    cfg.method = InverseMethod::Pseudoinverse;
    const Trajectory pinv = simulate_resolved_rate(arm, Eigen::Vector2d(0.3, 0.6), far, cfg);
    CHECK(pinv.qdot.cwiseAbs().maxCoeff() > 10.0 * bound);
}

TEST_CASE("variable damping equals the pseudoinverse away from singularities", "[singularities]") {
    const SerialChain arm = robots::planar({0.75, 0.5});
    ResolvedRateConfig cfg;
    cfg.space = TaskSpace::PlanarPosition;
    cfg.dt = 1.0 / 60.0;
    cfg.steps = 300;
    const Eigen::Matrix4d goal = translation({0.0, 1.0, 0.0});
    const Trajectory pinv = simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), goal, cfg);
    cfg.method = InverseMethod::DampedLeastSquares;
    cfg.damping_schedule = DampingSchedule::MinSingularValue;
    cfg.damping_threshold = 0.05;
    const Trajectory var = simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), goal, cfg);
    CHECK(near(var.q, pinv.q, 1e-12));
}

TEST_CASE("straight line through the PUMA wrist singularity: DLS avoids the wrist flip", "[singularities]") {
    // Section 5.3: the pseudoinverse spins joints 4 and 6 by about pi near q5 = 0; DLS passes through with
    // bounded rates and a small, temporary task error.
    const SerialChain arm = robots::puma560();
    Eigen::VectorXd qa(6);
    qa << 0.0, 0.6, -0.3, 0.5, 0.15, 0.0;
    const Eigen::Matrix4d Ta = arm.end_effector(qa);
    // Direction in which q5 decreases fastest for a pure translation (row 5 of J^-1).
    const Eigen::Vector3d g =
        Eigen::MatrixXd(geometric_jacobian(arm, qa).inverse()).block<1, 3>(4, 0).transpose();
    const Eigen::Vector3d dp = -0.15 * g.normalized();
    const double T = 4.0, dt = 0.005;
    const int steps = static_cast<int>(T / dt);
    std::vector<Eigen::Matrix4d> poses;
    Eigen::MatrixXd twists = Eigen::MatrixXd::Zero(steps + 1, 6);
    for (int k = 0; k <= steps; ++k) {
        const double s = 0.5 * (1.0 - std::cos(kPi * k / steps)),
                     sd = 0.5 * kPi / T * std::sin(kPi * k / steps);
        Eigen::Matrix4d Td = Ta;
        Td.block<3, 1>(0, 3) += s * dp;
        poses.push_back(Td);
        twists.block<1, 3>(k, 0) = (sd * dp).transpose();
    }
    ResolvedRateConfig cfg;
    cfg.gain = 5.0;
    cfg.dt = dt;
    const Trajectory pinv = simulate_pose_tracking(arm, qa, poses, twists, cfg);
    cfg.method = InverseMethod::DampedLeastSquares;
    cfg.damping = 0.05;
    const Trajectory dls = simulate_pose_tracking(arm, qa, poses, twists, cfg);
    CHECK(pinv.qdot.cwiseAbs().maxCoeff() > 10.0);    // wrist flip
    CHECK(std::abs(pinv.q(steps, 3) - qa(3)) > 2.5);  // joint 4 turned by about pi
    CHECK(dls.qdot.cwiseAbs().maxCoeff() < 1.0);      // no flip
    CHECK(dls.q(steps, 4) < 0.0);                     // q5 changed sign instead
    CHECK(dls.error_norm.maxCoeff() < 1e-2);          // small temporary error
    // Constant damping also slows convergence away from the singularity, so some error lingers at the end
    // (section 5.4). Damping only near the singularity removes most of it.
    cfg.damping_schedule = DampingSchedule::MinSingularValue;
    cfg.damping_threshold = 0.05;
    const Trajectory var = simulate_pose_tracking(arm, qa, poses, twists, cfg);
    CHECK(var.qdot.cwiseAbs().maxCoeff() < 1.0);
    CHECK(var.error_norm(steps) < 0.25 * dls.error_norm(steps));
}
