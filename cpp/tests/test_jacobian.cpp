// Chapter 3: Jacobians. References: the closed-form planar Jacobian of 1_theory/03_jacobian.md, central
// finite differences, and the product-of-exponentials space Jacobian (an independent construction).

#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/robots.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

TEST_CASE("planar two-link Jacobian matches the closed form", "[jacobian]") {
    // Worked example of section 3.2.
    const double l1 = 0.75, l2 = 0.5;
    const SerialChain arm = robots::planar({l1, l2});
    test_helpers::Rng rng;
    for (int k = 0; k < 20; ++k) {
        const Eigen::Vector2d q = rng.vector(2, -kPi, kPi);
        const double s1 = std::sin(q(0)), c1 = std::cos(q(0));
        const double s12 = std::sin(q(0) + q(1)), c12 = std::cos(q(0) + q(1));
        Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 2);
        expected.row(0) << -l1 * s1 - l2 * s12, -l2 * s12;
        expected.row(1) << l1 * c1 + l2 * c12, l2 * c12;
        expected.row(5) << 1.0, 1.0;
        CHECK(near(geometric_jacobian(arm, q), expected, 1e-14));
    }
}

TEST_CASE("geometric Jacobian agrees with central finite differences", "[jacobian]") {
    test_helpers::Rng rng(3);
    for (const SerialChain& arm : {robots::puma560(), robots::stanford(), robots::planar({0.4, 0.3, 0.2})}) {
        for (int t = 0; t < 10; ++t) {
            const Eigen::VectorXd q = rng.vector(arm.dof(), -kPi, kPi);
            CHECK(near(geometric_jacobian(arm, q), numerical_jacobian(arm, q), 1e-8));
            for (int k = 0; k <= arm.dof(); ++k) {
                CHECK(near(link_jacobian(arm, q, k), numerical_jacobian(arm, q, 1e-6, k), 1e-8));
            }
        }
    }
}

TEST_CASE("link Jacobian has zero columns beyond the link", "[jacobian]") {
    const SerialChain arm = robots::puma560();
    const Eigen::VectorXd q = test_helpers::Rng(5).vector(6, -2, 2);
    const Eigen::MatrixXd J3 = link_jacobian(arm, q, 3);
    CHECK(J3.rightCols(3).isZero(0.0));
    CHECK(link_jacobian(arm, q, 0).isZero(0.0));
    // The link-3 Jacobian equals the end-effector Jacobian of the arm cut after link 3.
    const std::vector<DHLink> first3(arm.links().begin(), arm.links().begin() + 3);
    CHECK(near(J3.leftCols(3), geometric_jacobian(SerialChain(first3), q.head(3)), 1e-14));
}

TEST_CASE("prismatic column is the joint axis with no angular part", "[jacobian]") {
    const SerialChain arm = robots::stanford();
    const Eigen::VectorXd q = test_helpers::Rng(6).vector(6, -1, 1);
    const auto frames = arm.frames(q);
    const Eigen::MatrixXd J = geometric_jacobian(arm, q);
    CHECK(near(J.block<3, 1>(0, 2), frames[2].block<3, 1>(0, 2), 1e-15));
    CHECK(J.block<3, 1>(3, 2).isZero(0.0));
}

TEST_CASE("geometric Jacobian equals the shifted space Jacobian of the POE chain", "[jacobian]") {
    // Eq. (3.7): J = [I, -[p_e]x; 0, I] J_s, and J_b = Ad_{T^-1} J_s.
    test_helpers::Rng rng(7);
    for (const SerialChain& arm : {robots::puma560(), robots::stanford()}) {
        const PoeChain poe = to_poe(arm);
        for (int t = 0; t < 10; ++t) {
            const Eigen::VectorXd q = rng.vector(arm.dof(), -kPi, kPi);
            const Eigen::Matrix4d T = arm.end_effector(q);
            Matrix6d shift = Matrix6d::Identity();
            shift.topRightCorner<3, 3>() = -skew(T.block<3, 1>(0, 3));
            CHECK(near(geometric_jacobian(arm, q), shift * space_jacobian(poe, q), 1e-12));
            CHECK(near(body_jacobian(poe, q), adjoint(inverse_transform(T)) * space_jacobian(poe, q), 1e-12));
        }
    }
}

TEST_CASE("body Jacobian gives the body twist", "[jacobian]") {
    // T^-1 dT/dt = [J_b qdot], checked by finite differences along a random joint velocity.
    const SerialChain arm = robots::puma560();
    const PoeChain poe = to_poe(arm);
    test_helpers::Rng rng(8);
    const Eigen::VectorXd q = rng.vector(6, -2, 2), qd = rng.vector(6, -1, 1);
    const double h = 1e-6;
    const Eigen::Matrix4d dT = (poe_space(poe, q + h * qd) - poe_space(poe, q - h * qd)) / (2 * h);
    const Vector6d Vb = twist_vee(inverse_transform(poe_space(poe, q)) * dT);
    CHECK(near(Vb, body_jacobian(poe, q) * qd, 1e-8));
}

TEST_CASE("analytic Jacobian matches finite differences of roll-pitch-yaw", "[jacobian]") {
    const SerialChain arm = robots::puma560();
    test_helpers::Rng rng(9);
    int checked = 0;
    while (checked < 10) {
        const Eigen::VectorXd q = rng.vector(6, -kPi, kPi);
        const Vector6d x = pose_rpy(arm.end_effector(q));
        if (std::abs(std::cos(x(4))) < 0.2) continue;  // stay away from the representation singularity
        CHECK(near(analytic_jacobian(arm, q), numerical_analytic_jacobian(arm, q), 1e-7));
        ++checked;
    }
}

TEST_CASE("rpy rate matrix maps angle rates to angular velocity", "[jacobian]") {
    const Eigen::Vector3d rpy(0.3, -0.5, 1.1), rate(0.2, -0.4, 0.7);
    const double h = 1e-6;
    const Eigen::Matrix3d Rp = rotation_from_rpy(rpy + h * rate), Rm = rotation_from_rpy(rpy - h * rate);
    CHECK(near(rpy_rate_matrix(rpy) * rate, log_so3(Rp * Rm.transpose()) / (2 * h), 1e-9));
    CHECK(std::abs(rpy_rate_matrix(rpy).determinant() - std::cos(rpy.y())) < 1e-15);
}
