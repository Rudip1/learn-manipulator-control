// Chapter 7: task-priority control. References: the minimum-norm solution of the stacked system (compatible
// tasks), a constrained least-squares problem solved through an orthonormal null-space basis (conflicting
// tasks), the closed-form algorithmic singularity of the three-link arm, and the task models checked against
// the chain's own kinematics.

#include <Eigen/LU>
#include <Eigen/QR>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/redundancy.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"
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

/// Orthonormal basis of ker A, computed by LU and orthonormalised by QR (independent of the pseudoinverse).
Eigen::MatrixXd orthonormal_kernel(const Eigen::MatrixXd& A) {
    const Eigen::MatrixXd Z = Eigen::FullPivLU<Eigen::MatrixXd>(A).kernel();
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(Z);
    return qr.householderQ() * Eigen::MatrixXd::Identity(Z.rows(), Z.cols());
}

}  // namespace

TEST_CASE("compatible tasks: the recursion gives the minimum-norm solution of the stacked system", "[priority]") {
    // Section 7.2: with sum m_i <= n and a full-rank stack every task is met exactly.
    test_helpers::Rng rng(71);
    const int n = 7;
    std::vector<PriorityLevel> levels = {{random_matrix(rng, 2, n), rng.vector(2, -1, 1)},
                                         {random_matrix(rng, 3, n), rng.vector(3, -1, 1)},
                                         {random_matrix(rng, 1, n), rng.vector(1, -1, 1)}};
    const Eigen::VectorXd u = task_priority_velocity(levels, n, PriorityConfig{});
    Eigen::MatrixXd J(6, n);
    Eigen::VectorXd v(6);
    J << levels[0].J, levels[1].J, levels[2].J;
    v << levels[0].v, levels[1].v, levels[2].v;
    CHECK(near(J * u, v, 1e-10));
    CHECK(near(u, pseudoinverse(J) * v, 1e-10));
    // Any order gives the same answer when the tasks are compatible.
    std::swap(levels[0], levels[2]);
    CHECK(near(task_priority_velocity(levels, n, PriorityConfig{}), u, 1e-10));
}

TEST_CASE("conflicting tasks: task 1 exact, task 2 in the least-squares sense inside ker J1", "[priority]") {
    // Section 7.3. Reference: u = J1^+ v1 + Z y with Z an orthonormal basis of ker J1 and y the minimum-norm
    // least-squares solution of (J2 Z) y = v2 - J2 J1^+ v1.
    test_helpers::Rng rng(72);
    const int n = 4;
    const Eigen::MatrixXd J1 = random_matrix(rng, 2, n), J2 = random_matrix(rng, 3, n);
    const Eigen::VectorXd v1 = rng.vector(2, -1, 1), v2 = rng.vector(3, -1, 1);
    const Eigen::VectorXd u = task_priority_velocity({{J1, v1}, {J2, v2}}, n, PriorityConfig{});
    const Eigen::MatrixXd Z = orthonormal_kernel(J1);
    const Eigen::VectorXd u1 = pseudoinverse(J1) * v1;
    const Eigen::VectorXd y = (J2 * Z).completeOrthogonalDecomposition().solve(v2 - J2 * u1);
    CHECK(near(J1 * u, v1, 1e-10));
    CHECK(near(u, u1 + Z * y, 1e-10));
    CHECK((J2 * u - v2).norm() > 1e-3);  // 5 equations, 4 unknowns: task 2 cannot be met
    // Swapping the priorities makes task 2 the exact one.
    const Eigen::VectorXd w = task_priority_velocity({{J2, v2}, {J1, v1}}, n, PriorityConfig{});
    CHECK((J2 * w - v2).norm() < (J2 * u - v2).norm());
}

TEST_CASE("hierarchy projector spans the common null space of the higher tasks", "[priority]") {
    test_helpers::Rng rng(73);
    const int n = 6;
    const std::vector<PriorityLevel> levels = {{random_matrix(rng, 2, n), Eigen::VectorXd::Zero(2)},
                                               {random_matrix(rng, 2, n), Eigen::VectorXd::Zero(2)}};
    const Eigen::MatrixXd P2 = hierarchy_projector(levels, n, 2);
    Eigen::MatrixXd J(4, n);
    J << levels[0].J, levels[1].J;
    CHECK(near(P2, null_space_projector(J), 1e-10));
    CHECK(near(hierarchy_projector(levels, n, 1), null_space_projector(levels[0].J), 1e-12));
}

TEST_CASE("singularity-robust method: task 1 exact, task 2 approximate even when compatible", "[priority]") {
    // Eq. (7.10) and section 7.5.
    test_helpers::Rng rng(74);
    const int n = 6;
    const std::vector<PriorityLevel> levels = {{random_matrix(rng, 2, n), rng.vector(2, -1, 1)},
                                               {random_matrix(rng, 2, n), rng.vector(2, -1, 1)}};
    PriorityConfig sr;
    sr.method = PriorityMethod::SingularityRobust;
    const Eigen::VectorXd u = task_priority_velocity(levels, n, sr);
    CHECK(near(levels[0].J * u, levels[0].v, 1e-10));
    CHECK((levels[1].J * u - levels[1].v).norm() > 1e-3);
    const Eigen::VectorXd r = task_priority_velocity(levels, n, PriorityConfig{});
    CHECK(near(levels[1].J * r, levels[1].v, 1e-10));
}

TEST_CASE("algorithmic singularity of tip position over joint 1 is at q3 = 0", "[priority]") {
    // Section 7.4: for the planar 3R arm, task 1 = tip (x, y), task 2 = q1. The augmented Jacobian J2 N1 has
    // norm l2 l3 |sin q3| / |n| with n = a x b the null-space direction, zero at q3 = 0 although J1 has rank 2
    // and J2 rank 1.
    const double l1 = 0.5, l2 = 0.4, l3 = 0.3;
    const SerialChain arm = robots::planar({l1, l2, l3});
    Eigen::RowVector3d J2(1.0, 0.0, 0.0);
    for (double q3 : {0.8, 0.3, 0.05, 0.0}) {
        const Eigen::Vector3d q(0.3, 0.7, q3);
        const Eigen::MatrixXd J1 = geometric_jacobian(arm, q).topRows(2);
        const Eigen::Vector3d nvec = Eigen::Vector3d(J1.row(0).transpose()).cross(Eigen::Vector3d(J1.row(1).transpose()));
        const double expected = l2 * l3 * std::abs(std::sin(q3)) / nvec.norm();
        const Eigen::MatrixXd J_bar = J2 * null_space_projector(J1);
        CHECK(std::abs(J_bar.norm() - expected) < 1e-12);
        CHECK(singular_values(J1)(1) > 0.1);  // the arm itself is far from singular
    }
}

TEST_CASE("DLS keeps the recursion bounded through the algorithmic singularity", "[priority]") {
    // Section 7.4: hold the tip still and drive q1 to 1.7 rad. Along this self-motion manifold q1 is at most
    // 1.479 rad, reached at q3 = 0, so the goal pulls the arm into the algorithmic singularity.
    const SerialArmModel model(robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(0.3, 0.7, 0.6);
    auto run = [&](PriorityMethod method, InverseMethod inverse) {
        auto tip = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, model.frame_pose(q0, -1));
        auto joint = std::make_shared<JointTask>("q1", 0, 1.7);
        tip->set_gain(2.0);
        joint->set_gain(2.0);
        PriorityConfig config;
        config.method = method;
        config.inverse = inverse;
        config.damping = 0.05;
        return simulate_task_priority(model, q0, {tip, joint}, config, 0.01, 800);
    };
    const PriorityTrajectory pinv = run(PriorityMethod::Recursive, InverseMethod::Pseudoinverse);
    const PriorityTrajectory dls = run(PriorityMethod::Recursive, InverseMethod::DampedLeastSquares);
    const PriorityTrajectory sr = run(PriorityMethod::SingularityRobust, InverseMethod::Pseudoinverse);
    const double peak_pinv = pinv.u.cwiseAbs().maxCoeff(), peak_dls = dls.u.cwiseAbs().maxCoeff();
    CHECK(peak_pinv > 100.0);                        // 680 rad/s
    CHECK(peak_pinv > 10.0 * peak_dls);              // DLS: 10.6 rad/s
    CHECK(pinv.error_norm.col(0).maxCoeff() > 0.1);  // ... and the primary task is lost (1.4 m)
    CHECK(dls.error_norm.col(0).maxCoeff() < 1e-2);  // DLS holds the tip within 1 cm
    CHECK(std::abs(dls.q(800, 0) - 1.479) < 0.01);   // and gets q1 as far as it can go
    CHECK(sr.u.cwiseAbs().maxCoeff() < 1.0);         // singularity-robust: slow and smooth,
    CHECK(sr.q(800, 0) < dls.q(800, 0) - 0.01);      // but stops short of the reachable maximum
}

TEST_CASE("task models match the chain's kinematics", "[priority]") {
    const SerialChain chain = robots::puma560();
    const SerialArmModel model(chain);
    Eigen::VectorXd q(6);
    q << 0.1, 0.5, -0.4, 0.3, 0.7, -0.2;
    Eigen::Matrix4d desired = chain.end_effector(q);
    desired.block<3, 1>(0, 3) += Eigen::Vector3d(0.01, -0.02, 0.03);
    FrameTask task("pos+yaw", -1, {0, 1, 2, 5}, desired);
    task.update(model, q);
    const Eigen::MatrixXd J = geometric_jacobian(chain, q);
    CHECK(near(task.jacobian().topRows(3), J.topRows(3), 1e-12));
    CHECK(near(task.jacobian().row(3), J.row(5), 1e-12));
    CHECK(near(task.error().head(3), Eigen::Vector3d(0.01, -0.02, 0.03), 1e-12));
    CHECK(std::abs(task.error()(3)) < 1e-12);
    task.set_gain(2.0);
    CHECK(near(task.velocity(), 2.0 * task.error(), 1e-15));
    // A frame inside the chain: DH frame 3 (wrist centre of the PUMA).
    FrameTask wrist("wrist", 3, {0, 1, 2}, Eigen::Matrix4d::Identity());
    wrist.update(model, q);
    CHECK(near(wrist.jacobian(), link_jacobian(chain, q, 3).topRows(3), 1e-12));
    JointTask joint("q5", 4, 1.0);
    joint.update(model, q);
    CHECK(joint.jacobian()(0, 4) == 1.0);
    CHECK(std::abs(joint.error()(0) - 0.3) < 1e-12);
}

TEST_CASE("simulated hierarchy reaches compatible goals", "[priority]") {
    // Tip position (2) then joint 1 (1) on a planar 4R arm: 3 <= 4, both met.
    const SerialArmModel model(robots::planar({0.4, 0.35, 0.3, 0.25}));
    const Eigen::Vector4d q0(0.2, 0.5, 0.4, 0.3);
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 0.6, 0.5, 0.0;
    auto tip = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
    auto joint = std::make_shared<JointTask>("q1", 0, 0.0);
    tip->set_gain(2.0);
    joint->set_gain(2.0);
    const PriorityTrajectory r = simulate_task_priority(model, q0, {tip, joint}, PriorityConfig{}, 0.01, 800);
    CHECK(r.error_norm(800, 0) < 1e-6);
    CHECK(r.error_norm(800, 1) < 1e-6);
    CHECK(std::abs(r.q(800, 0)) < 1e-6);
}
