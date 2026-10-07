#include "manipulator_control/transforms.hpp"

#include <algorithm>
#include <cmath>

namespace manipulator_control {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Below this angle the Rodrigues coefficients are evaluated by their Taylor series (section 1.4).
constexpr double kSmallAngle = 1e-4;

// sin(t)/t
double coeff_a(double t) { return t < kSmallAngle ? 1.0 - t * t / 6.0 : std::sin(t) / t; }
// (1 - cos(t))/t^2
double coeff_b(double t) { return t < kSmallAngle ? 0.5 - t * t / 24.0 : (1.0 - std::cos(t)) / (t * t); }
// (t - sin(t))/t^3
double coeff_c(double t) {
    return t < kSmallAngle ? 1.0 / 6.0 - t * t / 120.0 : (t - std::sin(t)) / (t * t * t);
}

}  // namespace

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S << 0.0, -w.z(), w.y(),  //
        w.z(), 0.0, -w.x(),   //
        -w.y(), w.x(), 0.0;
    return S;
}

Eigen::Vector3d unskew(const Eigen::Matrix3d& S) {
    return 0.5 * Eigen::Vector3d(S(2, 1) - S(1, 2), S(0, 2) - S(2, 0), S(1, 0) - S(0, 1));
}

Eigen::Matrix3d rot_x(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,  //
        0.0, c, -s,      //
        0.0, s, c;
    return R;
}

Eigen::Matrix3d rot_y(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    Eigen::Matrix3d R;
    R << c, 0.0, s,     //
        0.0, 1.0, 0.0,  //
        -s, 0.0, c;
    return R;
}

Eigen::Matrix3d rot_z(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    Eigen::Matrix3d R;
    R << c, -s, 0.0,  //
        s, c, 0.0,    //
        0.0, 0.0, 1.0;
    return R;
}

bool is_rotation(const Eigen::Matrix3d& R, double tol) {
    return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < tol &&
           std::abs(R.determinant() - 1.0) < tol;
}

Eigen::Matrix3d exp_so3(const Eigen::Vector3d& phi) {
    // Eq. (1.5) with the angle folded into W = [phi]x: R = I + sin(t)/t W + (1 - cos(t))/t^2 W^2.
    const double t = phi.norm();
    const Eigen::Matrix3d W = skew(phi);
    return Eigen::Matrix3d::Identity() + coeff_a(t) * W + coeff_b(t) * W * W;
}

Eigen::Vector3d log_so3(const Eigen::Matrix3d& R) {
    // theta from atan2 of (sin, cos) rather than acos of the trace alone: acos amplifies rounding near
    // theta = pi, and the error then reappears in the division by sin(theta).
    const Eigen::Vector3d axis_sin = unskew(R);  // = sin(t) * w_hat, eq. (1.6)
    const double t = std::atan2(axis_sin.norm(), 0.5 * (R.trace() - 1.0));
    if (t < 1e-8) {
        return axis_sin;  // first-order term
    }
    if (t > 0.75 * kPi) {
        // Towards theta = pi the skew part becomes too small to give an accurate direction. The symmetric
        // part of (1.5) is cos(t) I + (1 - cos(t)) w w^T, so w w^T can be read from it (section 1.1).
        const double cos_t = 0.5 * (R.trace() - 1.0);
        const Eigen::Matrix3d B =
            (0.5 * (R + R.transpose()) - cos_t * Eigen::Matrix3d::Identity()) / (1.0 - cos_t);
        Eigen::Index j = 0;
        B.diagonal().maxCoeff(&j);
        Eigen::Vector3d w = B.col(j) / std::sqrt(B(j, j));
        if (w.dot(axis_sin) < 0.0) {
            w = -w;
        }
        return w.normalized() * t;
    }
    return axis_sin * (t / std::sin(t));
}

Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d& rpy) {
    return rot_z(rpy.z()) * rot_y(rpy.y()) * rot_x(rpy.x());
}

Eigen::Vector3d rpy_from_rotation(const Eigen::Matrix3d& R) {
    const double pitch = std::atan2(-R(2, 0), std::hypot(R(0, 0), R(1, 0)));
    if (std::hypot(R(0, 0), R(1, 0)) < 1e-12) {
        // Gimbal lock: only yaw -+ roll is observable; put all of it in yaw.
        const double yaw = std::atan2(-R(0, 1), R(1, 1));
        return {0.0, pitch, yaw};
    }
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const double roll = std::atan2(R(2, 1), R(2, 2));
    return {roll, pitch, yaw};
}

Eigen::Matrix4d make_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = p;
    return T;
}

Eigen::Matrix4d translation(const Eigen::Vector3d& p) {
    return make_transform(Eigen::Matrix3d::Identity(), p);
}

Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d Rt = T.topLeftCorner<3, 3>().transpose();
    return make_transform(Rt, -Rt * T.topRightCorner<3, 1>());
}

Eigen::Matrix4d twist_hat(const Vector6d& V) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    M.topLeftCorner<3, 3>() = skew(V.tail<3>());
    M.topRightCorner<3, 1>() = V.head<3>();
    return M;
}

Vector6d twist_vee(const Eigen::Matrix4d& V_hat) {
    Vector6d V;
    V << V_hat.topRightCorner<3, 1>(), unskew(V_hat.topLeftCorner<3, 3>());
    return V;
}

Matrix6d adjoint(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d p = T.topRightCorner<3, 1>();
    Matrix6d Ad = Matrix6d::Zero();
    Ad.topLeftCorner<3, 3>() = R;
    Ad.topRightCorner<3, 3>() = skew(p) * R;
    Ad.bottomRightCorner<3, 3>() = R;
    return Ad;
}

Vector6d screw_axis(const Eigen::Vector3d& q, const Eigen::Vector3d& w_hat, double h) {
    Vector6d S;
    S << -w_hat.cross(q) + h * w_hat, w_hat;
    return S;
}

Eigen::Matrix4d exp_se3(const Vector6d& xi) {
    // Eq. (1.13), algorithm in section 1.4.
    const Eigen::Vector3d nu = xi.head<3>();
    const Eigen::Vector3d phi = xi.tail<3>();
    const double t = phi.norm();
    const Eigen::Matrix3d W = skew(phi);
    const Eigen::Matrix3d W2 = W * W;
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R = I + coeff_a(t) * W + coeff_b(t) * W2;
    const Eigen::Vector3d p = (I + coeff_b(t) * W + coeff_c(t) * W2) * nu;
    return make_transform(R, p);
}

Vector6d log_se3(const Eigen::Matrix4d& T) {
    // Eq. (1.14).
    const Eigen::Vector3d phi = log_so3(T.topLeftCorner<3, 3>());
    const double t = phi.norm();
    const Eigen::Matrix3d W = skew(phi);
    // (1 - t sin t / (2 (1 - cos t))) / t^2, with its Taylor series 1/12 + t^2/720 near zero.
    const double d = t < kSmallAngle ? 1.0 / 12.0 + t * t / 720.0
                                     : (1.0 - t * std::sin(t) / (2.0 * (1.0 - std::cos(t)))) / (t * t);
    const Eigen::Matrix3d G_inv = Eigen::Matrix3d::Identity() - 0.5 * W + d * W * W;
    Vector6d xi;
    xi << G_inv * T.topRightCorner<3, 1>(), phi;
    return xi;
}

}  // namespace manipulator_control
