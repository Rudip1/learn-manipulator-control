// Chapter 1: rotations, transforms, twists. References: closed forms, the worked example in
// 1_theory/01_rigid_body_transforms.md, and Eigen::AngleAxis (an independent Rodrigues implementation).

#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "manipulator_control/transforms.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

TEST_CASE("skew matrix implements the cross product", "[transforms]") {
    test_helpers::Rng rng;
    for (int k = 0; k < 20; ++k) {
        const Eigen::Vector3d a = rng.vector(3, -2, 2), b = rng.vector(3, -2, 2);
        CHECK(near(skew(a) * b, a.cross(b)));
        CHECK(near(skew(a).transpose(), -skew(a)));
        CHECK(near(unskew(skew(a)), a));
    }
}

TEST_CASE("elementary rotations match their closed forms", "[transforms]") {
    Eigen::Matrix3d Rz90;
    Rz90 << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    CHECK(near(rot_z(kPi / 2), Rz90, 1e-15));
    // Rx(90deg) maps y to z, Ry(90deg) maps z to x.
    CHECK(near(rot_x(kPi / 2) * Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), 1e-15));
    CHECK(near(rot_y(kPi / 2) * Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 1e-15));
    CHECK(is_rotation(rot_x(0.3) * rot_y(-1.2) * rot_z(2.0)));
    CHECK_FALSE(is_rotation(2.0 * Eigen::Matrix3d::Identity()));
}

TEST_CASE("exp_so3 agrees with Eigen::AngleAxis", "[transforms]") {
    test_helpers::Rng rng(7);
    for (int k = 0; k < 50; ++k) {
        const Eigen::Vector3d axis = rng.vector(3, -1, 1).normalized();
        const double angle = rng.uniform(-3.0, 3.0);
        const Eigen::Matrix3d expected = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        CHECK(near(exp_so3(axis * angle), expected, 1e-12));
    }
    // Small angles use the Taylor branch.
    const Eigen::Vector3d tiny(1e-7, -2e-7, 3e-7);
    CHECK(near(exp_so3(tiny), Eigen::AngleAxisd(tiny.norm(), tiny.normalized()).toRotationMatrix(), 1e-15));
    CHECK(near(exp_so3(Eigen::Vector3d::Zero()), Eigen::Matrix3d::Identity(), 0.0));
}

TEST_CASE("log_so3 inverts exp_so3 including theta near 0 and pi", "[transforms]") {
    test_helpers::Rng rng(11);
    for (int k = 0; k < 50; ++k) {
        const Eigen::Vector3d axis = rng.vector(3, -1, 1).normalized();
        const Eigen::Vector3d phi = axis * rng.uniform(0.0, kPi - 1e-3);
        CHECK(near(log_so3(exp_so3(phi)), phi, 1e-10));
    }
    for (double t : {0.0, 1e-10, 1e-6, kPi - 1e-5, kPi - 1e-7, kPi}) {
        const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
        const Eigen::Matrix3d R = exp_so3(axis * t);
        const Eigen::Vector3d phi = log_so3(R);
        CHECK(std::abs(phi.norm() - t) < 1e-6);
        CHECK(near(exp_so3(phi), R, 1e-9));  // same rotation (at pi the sign of the axis is free)
    }
}

TEST_CASE("roll-pitch-yaw round trip and gimbal lock", "[transforms]") {
    const Eigen::Vector3d rpy(0.3, -0.7, 2.1);
    const Eigen::Matrix3d R = rotation_from_rpy(rpy);
    CHECK(near(R, rot_z(2.1) * rot_y(-0.7) * rot_x(0.3)));
    CHECK(near(rpy_from_rotation(R), rpy, 1e-12));
    // At pitch = pi/2 only yaw - roll is defined: two different triples give the same matrix.
    const Eigen::Matrix3d A = rotation_from_rpy(Eigen::Vector3d(0.2, kPi / 2, 0.5));
    const Eigen::Matrix3d B = rotation_from_rpy(Eigen::Vector3d(0.0, kPi / 2, 0.3));
    CHECK(near(A, B, 1e-12));
    CHECK(near(rotation_from_rpy(rpy_from_rotation(A)), A, 1e-12));
}

TEST_CASE("homogeneous transform inverse and composition", "[transforms]") {
    const Eigen::Matrix4d T =
        make_transform(rotation_from_rpy(Eigen::Vector3d(0.1, 0.2, 0.3)), {1.0, -2.0, 0.5});
    CHECK(near(inverse_transform(T) * T, Eigen::Matrix4d::Identity(), 1e-14));
    CHECK(near(inverse_transform(T), Eigen::Matrix4d(T.inverse()), 1e-14));
    CHECK(near(translation({1, 2, 3}).topRightCorner<3, 1>(), Eigen::Vector3d(1, 2, 3)));
}

TEST_CASE("adjoint maps se(3) matrices by conjugation", "[transforms]") {
    test_helpers::Rng rng(3);
    const Eigen::Matrix4d T = make_transform(exp_so3(rng.vector(3, -1, 1)), rng.vector(3, -1, 1));
    const Vector6d V = rng.vector(6, -1, 1);
    // Eq. (1.11): [Ad_T V] = T [V] T^-1.
    CHECK(near(twist_hat(adjoint(T) * V), T * twist_hat(V) * inverse_transform(T), 1e-12));
    CHECK(near(twist_vee(twist_hat(V)), V));
}

TEST_CASE("worked example: quarter turn about a vertical line through (1,0,0)", "[transforms]") {
    // Section 1.3 of 1_theory/01_rigid_body_transforms.md.
    const Vector6d S = screw_axis({1.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ());
    CHECK(near(S, (Vector6d() << 0, -1, 0, 0, 0, 1).finished()));
    const Eigen::Matrix4d T = exp_se3(S * kPi / 2);
    CHECK(near(T.topLeftCorner<3, 3>(), rot_z(kPi / 2), 1e-15));
    CHECK(near(T.topRightCorner<3, 1>(), Eigen::Vector3d(1.0, -1.0, 0.0), 1e-15));
}

TEST_CASE("exp_se3 of a rotation about an offset axis and of a screw with pitch", "[transforms]") {
    test_helpers::Rng rng(5);
    for (int k = 0; k < 30; ++k) {
        const Eigen::Vector3d q = rng.vector(3, -1, 1);
        const Eigen::Vector3d w = rng.vector(3, -1, 1).normalized();
        const double h = rng.uniform(-0.5, 0.5), t = rng.uniform(-3.0, 3.0);
        const Eigen::Matrix4d T = exp_se3(screw_axis(q, w, h) * t);
        const Eigen::Matrix3d R = Eigen::AngleAxisd(t, w).toRotationMatrix();
        // Closed form of a screw motion: rotate about the line through q, then advance h*t along it.
        const Eigen::Vector3d p = (Eigen::Matrix3d::Identity() - R) * q + h * t * w;
        CHECK(near(T.topLeftCorner<3, 3>(), R, 1e-12));
        CHECK(near(T.topRightCorner<3, 1>(), p, 1e-12));
    }
    // Pure translation.
    const Vector6d xi = (Vector6d() << 0.3, -0.2, 0.1, 0, 0, 0).finished();
    CHECK(near(exp_se3(xi), translation({0.3, -0.2, 0.1}), 0.0));
}

TEST_CASE("log_se3 inverts exp_se3", "[transforms]") {
    test_helpers::Rng rng(9);
    for (int k = 0; k < 50; ++k) {
        Vector6d xi = rng.vector(6, -1, 1);
        xi.tail<3>() = xi.tail<3>().normalized() * rng.uniform(0.0, kPi - 1e-3);
        CHECK(near(log_se3(exp_se3(xi)), xi, 1e-9));
    }
    const Vector6d small = (Vector6d() << 0.1, 0.2, 0.3, 1e-9, -1e-9, 2e-9).finished();
    CHECK(near(log_se3(exp_se3(small)), small, 1e-12));
}
