// Chapter 2: forward kinematics. References: closed forms for the planar arm, the hand-worked PUMA home pose
// in 1_theory/02_forward_kinematics.md, and agreement between the DH and product-of-exponentials
// formulations.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <stdexcept>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/robots.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

TEST_CASE("dh_transform is Rz(theta) Tz(d) Tx(a) Rx(alpha)", "[kinematics]") {
    test_helpers::Rng rng;
    for (int k = 0; k < 20; ++k) {
        const double a = rng.uniform(-1, 1), alpha = rng.uniform(-3, 3), d = rng.uniform(-1, 1),
                     theta = rng.uniform(-3, 3);
        const Eigen::Matrix4d expected = make_transform(rot_z(theta), Eigen::Vector3d::Zero()) *
                                         translation({0, 0, d}) * translation({a, 0, 0}) *
                                         make_transform(rot_x(alpha), Eigen::Vector3d::Zero());
        CHECK(near(dh_transform(a, alpha, d, theta), expected, 1e-14));
    }
}

TEST_CASE("planar two-link arm matches the closed form", "[kinematics]") {
    // Worked example of section 2.2: p = (l1 c1 + l2 c12, l1 s1 + l2 s12), heading q1 + q2.
    const double l1 = 0.75, l2 = 0.5;
    const SerialChain arm = robots::planar({l1, l2});
    test_helpers::Rng rng(2);
    for (int k = 0; k < 20; ++k) {
        const Eigen::Vector2d q = rng.vector(2, -kPi, kPi);
        const Eigen::Matrix4d T = arm.end_effector(q);
        const Eigen::Vector3d p(l1 * std::cos(q(0)) + l2 * std::cos(q(0) + q(1)),
                                l1 * std::sin(q(0)) + l2 * std::sin(q(0) + q(1)), 0.0);
        CHECK(near(T.topRightCorner<3, 1>(), p, 1e-14));
        CHECK(near(T.topLeftCorner<3, 3>(), rot_z(q(0) + q(1)), 1e-14));
    }
    const auto frames = arm.frames(Eigen::Vector2d(0.2, 0.5));
    REQUIRE(frames.size() == 4);  // base, link 1, link 2, end effector
    CHECK(near(frames[1].topRightCorner<3, 1>(), Eigen::Vector3d(l1 * std::cos(0.2), l1 * std::sin(0.2), 0)));
}

TEST_CASE("PUMA 560 home pose matches the hand computation", "[kinematics]") {
    // Section 2.2: at q = 0 the wrist centre is at (a2 + a3, -d3, d4) and the end-effector frame is aligned
    // with the base frame.
    const Eigen::Matrix4d T = robots::puma560().end_effector(Eigen::VectorXd::Zero(6));
    CHECK(near(T.topRightCorner<3, 1>(), Eigen::Vector3d(0.4318 + 0.0203, -0.15005, 0.4318), 1e-14));
    CHECK(near(T.topLeftCorner<3, 3>(), Eigen::Matrix3d::Identity(), 1e-14));
}

TEST_CASE("prismatic joint translates along the previous z axis", "[kinematics]") {
    const SerialChain arm = robots::stanford();
    Eigen::VectorXd q(6);
    q << 0.3, -0.4, 0.5, 0.1, 0.2, -0.3;
    const auto frames = arm.frames(q);
    Eigen::VectorXd q2 = q;
    q2(2) += 0.1;
    const Eigen::Vector3d dp =
        arm.end_effector(q2).topRightCorner<3, 1>() - frames.back().topRightCorner<3, 1>();
    CHECK(near(dp, 0.1 * frames[2].block<3, 1>(0, 2), 1e-14));
    CHECK(near(arm.end_effector(q2).topLeftCorner<3, 3>(), frames.back().topLeftCorner<3, 3>(), 1e-14));
}

TEST_CASE("base and tool transforms compose on both sides", "[kinematics]") {
    SerialChain arm = robots::puma560();
    const Eigen::VectorXd q = test_helpers::Rng(4).vector(6, -2, 2);
    const Eigen::Matrix4d T = arm.end_effector(q);
    const Eigen::Matrix4d B = make_transform(rot_z(0.7), {1.0, 2.0, 0.3});
    const Eigen::Matrix4d E = make_transform(rot_x(-0.4), {0.0, 0.0, 0.1});
    arm.set_base(B);
    arm.set_tool(E);
    CHECK(near(arm.end_effector(q), B * T * E, 1e-13));
}

TEST_CASE("wrong number of joint values throws", "[kinematics]") {
    CHECK_THROWS_AS(robots::puma560().end_effector(Eigen::VectorXd::Zero(5)), std::invalid_argument);
    CHECK_THROWS_AS(robots::puma560().link_transform(0, 0.0), std::out_of_range);
}

TEST_CASE("screw axes of the planar two-link arm", "[kinematics]") {
    // Exercise of the chapter 2 notebook: S1 = (0,0,0, 0,0,1), S2 = (0,-l1,0, 0,0,1), M = Tx(l1 + l2).
    const PoeChain poe = to_poe(robots::planar({0.75, 0.5}));
    REQUIRE(poe.screws.size() == 2);
    CHECK(near(poe.screws[0], (Vector6d() << 0, 0, 0, 0, 0, 1).finished()));
    CHECK(near(poe.screws[1], (Vector6d() << 0, -0.75, 0, 0, 0, 1).finished()));
    CHECK(near(poe.M, translation({1.25, 0, 0})));
}

TEST_CASE("DH and product of exponentials describe the same arm", "[kinematics]") {
    test_helpers::Rng rng(8);
    for (const SerialChain& arm : {robots::puma560(), robots::stanford(), robots::planar({0.4, 0.3, 0.2})}) {
        const PoeChain poe = to_poe(arm);
        for (int k = 0; k < 25; ++k) {
            const Eigen::VectorXd q = rng.vector(arm.dof(), -kPi, kPi);
            CHECK(near(poe_space(poe, q), arm.end_effector(q), 1e-12));
            CHECK(near(poe_body(poe, q), arm.end_effector(q), 1e-12));
        }
    }
}
