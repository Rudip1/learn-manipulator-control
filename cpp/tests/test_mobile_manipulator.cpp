// Chapter 9: mobile manipulators. References: the Jacobian against finite differences of the integrated
// motion, the unicycle arc against the circle it should trace and against many small Euler steps, the
// nonholonomic constraint, the weighted hierarchy against the weighted pseudoinverse of chapter 6, and the
// closed loop reaching beyond the arm.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "manipulator_control/inequality_tasks.hpp"
#include "manipulator_control/mobile_manipulator.hpp"
#include "manipulator_control/redundancy.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"
#include "manipulator_control/transforms.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::kPi;
using test_helpers::near;

namespace {

MobileManipulator make_robot() {
    Eigen::Matrix4d mount = Eigen::Matrix4d::Identity();
    mount.block<3, 1>(0, 3) << 0.1, 0.0, 0.2;
    return MobileManipulator(robots::anthropomorphic(), mount);
}

}  // namespace

TEST_CASE("mobile-manipulator Jacobian matches finite differences of the motion", "[mobile]") {
    // Column j is d(pose)/d(u_j): apply u = +-eps e_j for one second with the model's own integration.
    const MobileManipulator robot = make_robot();
    Eigen::VectorXd q(6);
    q << 0.4, -0.3, 0.7, 0.2, 0.5, -1.1;
    const double eps = 1e-6;
    for (int k : {-1, 0, 1, 2, 3, 4}) {
        const Eigen::MatrixXd J = robot.frame_jacobian(q, k);
        Eigen::MatrixXd fd(6, robot.control_size());
        for (int j = 0; j < robot.control_size(); ++j) {
            Eigen::VectorXd du = Eigen::VectorXd::Zero(robot.control_size());
            du(j) = eps;
            const Eigen::Matrix4d Tp = robot.frame_pose(robot.integrate(q, du, 1.0), k);
            const Eigen::Matrix4d Tm = robot.frame_pose(robot.integrate(q, -du, 1.0), k);
            fd.block<3, 1>(0, j) = (Tp.block<3, 1>(0, 3) - Tm.block<3, 1>(0, 3)) / (2 * eps);
            const Eigen::Matrix3d dR = Tp.topLeftCorner<3, 3>() * Tm.topLeftCorner<3, 3>().transpose();
            fd.block<3, 1>(3, j) = log_so3(dR) / (2 * eps);
        }
        CHECK(near(J, fd, 1e-7));
    }
}

TEST_CASE("unicycle integration follows the exact arc", "[mobile]") {
    // Eq. (9.6): with constant omega and v the base moves on a circle of radius v / omega.
    const Eigen::Vector3d eta0(0.5, -0.2, 0.3);
    const double omega = 0.8, v = 0.4, T = 2.0;
    const Eigen::Vector3d eta = integrate_unicycle(eta0, omega, v, T);
    const double R = v / omega;
    const Eigen::Vector2d centre(eta0(0) - R * std::sin(eta0(2)), eta0(1) + R * std::cos(eta0(2)));
    CHECK(std::abs((eta.head<2>() - centre).norm() - R) < 1e-12);
    CHECK(std::abs(eta(2) - (eta0(2) + omega * T)) < 1e-15);
    // Many small Euler steps converge to the same point.
    Eigen::Vector3d euler = eta0;
    const int N = 200000;
    for (int k = 0; k < N; ++k) {
        euler(0) += T / N * v * std::cos(euler(2));
        euler(1) += T / N * v * std::sin(euler(2));
        euler(2) += T / N * omega;
    }
    CHECK((euler - eta).norm() < 1e-5);
    // Straight line for omega = 0.
    CHECK(near(integrate_unicycle(eta0, 0.0, v, T), Eigen::Vector3d(0.5 + 0.8 * std::cos(0.3), -0.2 + 0.8 * std::sin(0.3), 0.3), 1e-12));
}

TEST_CASE("the base cannot move sideways", "[mobile]") {
    // Eq. (9.2): the lateral velocity of the base origin is zero for every command.
    const MobileManipulator robot = make_robot();
    test_helpers::Rng rng(91);
    for (int trial = 0; trial < 10; ++trial) {
        const Eigen::VectorXd q = rng.vector(6, -kPi, kPi);
        const Eigen::MatrixXd J = robot.frame_jacobian(q, 0);
        const Eigen::RowVector3d lateral(-std::sin(q(2)), std::cos(q(2)), 0.0);
        CHECK((lateral * J.topRows(3)).norm() < 1e-14);
        CHECK(J.rightCols(3).norm() == 0.0);  // the arm does not move the base
    }
}

TEST_CASE("weighted hierarchy equals the weighted pseudoinverse for one level", "[mobile]") {
    // Eq. (9.8) against eq. (6.6).
    const MobileManipulator robot = make_robot();
    Eigen::VectorXd q(6);
    q << 0.0, 0.0, 0.2, 0.1, 0.6, -1.2;
    const Eigen::MatrixXd J = robot.frame_jacobian(q, -1).topRows(3);
    const Eigen::Vector3d v(0.1, -0.2, 0.05);
    Eigen::VectorXd w(5);
    w << 10.0, 10.0, 1.0, 2.0, 1.0;
    PriorityConfig config;
    config.weights = w;
    const Eigen::VectorXd u = task_priority_velocity({{J, v}}, 5, config);
    CHECK(near(u, weighted_pseudoinverse(J, w.asDiagonal().toDenseMatrix()) * v, 1e-10));
    // A heavy base makes the base contribute less.
    const Eigen::VectorXd u_plain = task_priority_velocity({{J, v}}, 5, PriorityConfig{});
    CHECK(u.head(2).norm() < u_plain.head(2).norm());
}

TEST_CASE("the base drives the end effector to a point beyond the arm's reach", "[mobile]") {
    // Section 9.5: target 2.2 m away; the arm alone reaches 0.65 m.
    const MobileManipulator robot = make_robot();
    Eigen::VectorXd q0(6);
    q0 << 0.0, 0.0, 0.0, 0.0, 0.6, -1.2;
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 2.0, 1.0, 0.3;
    auto run = [&](const Eigen::VectorXd& weights, bool elbow_limit) {
        auto ee = std::make_shared<FrameTask>("ee", -1, std::vector<int>{0, 1, 2}, target);
        auto elbow = std::make_shared<JointLimitTask>("elbow", 4, -2.5, -0.5, 0.05, 0.15);
        elbow->set_gain(0.5);
        PriorityConfig config;
        config.inverse = InverseMethod::DampedLeastSquares;
        config.damping = 0.05;
        config.weights = weights;
        const TaskList tasks = elbow_limit ? TaskList{elbow, ee} : TaskList{ee};
        return simulate_task_priority(robot, q0, tasks, config, 0.01, 1500);
    };
    Eigen::VectorXd equal = Eigen::VectorXd::Ones(5), heavy_base = equal;
    heavy_base.head(2) *= 10.0;
    const PriorityTrajectory a = run(equal, false);
    const PriorityTrajectory b = run(heavy_base, false);
    const PriorityTrajectory c = run(heavy_base, true);
    CHECK(a.error_norm(1500, 0) < 1e-5);
    CHECK(c.error_norm(1500, 1) < 1e-4);
    CHECK(a.q.block(1500, 0, 1, 2).norm() > 1.5);  // the base has driven
    // Heavy base: the arm stretches out to its boundary singularity (elbow near 0) ...
    CHECK(std::abs(b.q(1500, 5)) < 0.05);
    // ... unless the elbow limit keeps it bent.
    CHECK(c.q.col(5).maxCoeff() < -0.5);
}
