#include "manipulator_control/jacobian.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace manipulator_control {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_to_pi(double a) { return std::remainder(a, 2.0 * kPi); }

}  // namespace

Eigen::MatrixXd geometric_jacobian(const std::vector<Eigen::Matrix4d>& frames,
                                   const std::vector<JointType>& types, int k) {
    const int n = static_cast<int>(types.size());
    if (static_cast<int>(frames.size()) < n + 1)
        throw std::invalid_argument("geometric_jacobian: too few frames");
    if (k < 0 || k >= static_cast<int>(frames.size()))
        throw std::out_of_range("geometric_jacobian: frame index " + std::to_string(k) + " out of range");
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    const Eigen::Vector3d p = frames[size_t(k)].block<3, 1>(0, 3);
    for (int i = 1; i <= std::min(k, n); ++i) {
        const Eigen::Vector3d z = frames[size_t(i - 1)].block<3, 1>(0, 2);
        const Eigen::Vector3d o = frames[size_t(i - 1)].block<3, 1>(0, 3);
        if (types[size_t(i - 1)] == JointType::Revolute) {
            J.block<3, 1>(0, i - 1) = z.cross(p - o);  // eq. (3.2)
            J.block<3, 1>(3, i - 1) = z;
        } else {
            J.block<3, 1>(0, i - 1) = z;  // eq. (3.2)
        }
    }
    return J;
}

Eigen::MatrixXd geometric_jacobian(const SerialChain& chain, const Eigen::VectorXd& q) {
    return link_jacobian(chain, q, chain.dof() + 1);
}

Eigen::MatrixXd link_jacobian(const SerialChain& chain, const Eigen::VectorXd& q, int k) {
    return geometric_jacobian(chain.frames(q), chain.joint_types(), k);
}

Eigen::MatrixXd space_jacobian(const PoeChain& chain, const Eigen::VectorXd& q) {
    const Eigen::Index n = Eigen::Index(chain.screws.size());
    if (q.size() != n) throw std::invalid_argument("space_jacobian: q has the wrong size");
    Eigen::MatrixXd J(6, n);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (Eigen::Index i = 0; i < n; ++i) {
        J.col(i) = adjoint(T) * chain.screws[size_t(i)];  // eq. (3.5)
        T = T * exp_se3(chain.screws[size_t(i)] * q(i));
    }
    return J;
}

Eigen::MatrixXd body_jacobian(const PoeChain& chain, const Eigen::VectorXd& q) {
    return adjoint(inverse_transform(poe_space(chain, q))) * space_jacobian(chain, q);  // eq. (3.6)
}

Eigen::Matrix3d rpy_rate_matrix(const Eigen::Vector3d& rpy) {
    // Columns: roll axis Rz(yaw) Ry(pitch) x, pitch axis Rz(yaw) y, yaw axis z. Eq. (3.8).
    const double cp = std::cos(rpy.y()), sp = std::sin(rpy.y());
    const double cy = std::cos(rpy.z()), sy = std::sin(rpy.z());
    Eigen::Matrix3d T;
    T << cy * cp, -sy, 0.0,  //
        sy * cp, cy, 0.0,    //
        -sp, 0.0, 1.0;
    return T;
}

Vector6d pose_rpy(const Eigen::Matrix4d& T) {
    Vector6d x;
    x << T.block<3, 1>(0, 3), rpy_from_rotation(T.topLeftCorner<3, 3>());
    return x;
}

Eigen::MatrixXd analytic_jacobian(const SerialChain& chain, const Eigen::VectorXd& q) {
    Eigen::MatrixXd J = geometric_jacobian(chain, q);
    const Eigen::Vector3d rpy = rpy_from_rotation(chain.end_effector(q).topLeftCorner<3, 3>());
    J.bottomRows<3>() = rpy_rate_matrix(rpy).inverse() * J.bottomRows<3>();  // eq. (3.9)
    return J;
}

Eigen::MatrixXd numerical_jacobian(const SerialChain& chain, const Eigen::VectorXd& q, double h, int k) {
    const int n = chain.dof();
    const size_t idx = k < 0 ? size_t(n + 1) : size_t(k);
    Eigen::MatrixXd J(6, n);
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd qp = q, qm = q;
        qp(i) += h;
        qm(i) -= h;
        const Eigen::Matrix4d Tp = chain.frames(qp).at(idx), Tm = chain.frames(qm).at(idx);
        J.block<3, 1>(0, i) = (Tp.block<3, 1>(0, 3) - Tm.block<3, 1>(0, 3)) / (2.0 * h);  // eq. (3.10)
        J.block<3, 1>(3, i) =
            log_so3(Tp.topLeftCorner<3, 3>() * Tm.topLeftCorner<3, 3>().transpose()) / (2.0 * h);
    }
    return J;
}

Eigen::MatrixXd numerical_analytic_jacobian(const SerialChain& chain, const Eigen::VectorXd& q, double h) {
    const int n = chain.dof();
    Eigen::MatrixXd J(6, n);
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd qp = q, qm = q;
        qp(i) += h;
        qm(i) -= h;
        Vector6d dx = pose_rpy(chain.end_effector(qp)) - pose_rpy(chain.end_effector(qm));
        for (int r = 3; r < 6; ++r) dx(r) = wrap_to_pi(dx(r));
        J.col(i) = dx / (2.0 * h);
    }
    return J;
}

}  // namespace manipulator_control
