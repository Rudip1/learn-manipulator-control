// Chapter 8: inequality tasks. References: the activation state machine worked by hand, finite differences
// of the obstacle distance, and closed-loop properties (limit kept, obstacle cleared, primary task reached,
// inactive tasks leaving the motion unchanged).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "manipulator_control/inequality_tasks.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"
#include "test_helpers.hpp"

using namespace manipulator_control;
using test_helpers::near;

namespace {

int switches(const Eigen::VectorXd& active) {
    int count = 0;
    for (Eigen::Index k = 1; k < active.size(); ++k) count += active(k) != active(k - 1);
    return count;
}

double max_distance_violation(const SerialArmModel& model, const Eigen::MatrixXd& q, int frame,
                              const Eigen::Vector2d& c, double radius) {
    double worst = -1e9;
    for (Eigen::Index k = 0; k < q.rows(); ++k) {
        const Eigen::Vector2d p = model.frame_pose(q.row(k).transpose(), frame).block<2, 1>(0, 3);
        worst = std::max(worst, radius - (p - c).norm());
    }
    return worst;  // > 0 means the obstacle was entered
}

}  // namespace

TEST_CASE("activation switches on within alpha and off beyond delta", "[inequality]") {
    // Eq. (8.3) with bounds [-1, 1], alpha = 0.1, delta = 0.3, walked up to the upper bound and back.
    const double lo = -1.0, hi = 1.0, a = 0.1, d = 0.3;
    int s = 0;
    s = next_activation(s, 0.85, lo, hi, a, d);
    CHECK(s == 0);
    s = next_activation(s, 0.91, lo, hi, a, d);
    CHECK(s == -1);  // within alpha of the upper bound: push down
    s = next_activation(s, 0.80, lo, hi, a, d);
    CHECK(s == -1);  // inside the hysteresis band: stay on
    s = next_activation(s, 0.69, lo, hi, a, d);
    CHECK(s == 0);  // delta away: off
    s = next_activation(s, -0.95, lo, hi, a, d);
    CHECK(s == +1);  // lower bound
    s = next_activation(s, -0.75, lo, hi, a, d);
    CHECK(s == +1);
    s = next_activation(s, -0.65, lo, hi, a, d);
    CHECK(s == 0);
}

TEST_CASE("obstacle distance Jacobian matches finite differences", "[inequality]") {
    const SerialArmModel model(robots::puma560());
    Eigen::VectorXd q(6);
    q << 0.2, 0.4, -0.3, 0.5, 0.6, -0.1;
    const Eigen::Vector3d c(0.3, 0.2, 0.4);
    ObstacleTask task("obstacle", 3, c, 0.05, 0.02, 0.05);
    task.update(model, q);
    Eigen::RowVectorXd fd(6);
    for (int i = 0; i < 6; ++i) {
        Eigen::VectorXd qp = q, qm = q;
        qp(i) += 1e-6;
        qm(i) -= 1e-6;
        const double dp = (model.frame_pose(qp, 3).block<3, 1>(0, 3) - c).norm();
        const double dm = (model.frame_pose(qm, 3).block<3, 1>(0, 3) - c).norm();
        fd(i) = (dp - dm) / 2e-6;
    }
    CHECK(near(task.jacobian(), fd, 1e-8));
    CHECK(std::abs(task.value() - (model.frame_pose(q, 3).block<3, 1>(0, 3) - c).norm()) < 1e-14);
}

TEST_CASE("joint-limit task keeps q3 inside its limit while the tip reaches the target", "[inequality]") {
    // Section 8.4: the target of chapter 6; the minimum-norm motion takes q3 to 1.859 rad, the limit is 1.7.
    const SerialArmModel model(robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(1.2, -1.5, 1.6);
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 0.6, 0.5, 0.0;
    auto run = [&](double delta, bool with_limit) {
        auto limit = std::make_shared<JointLimitTask>("q3 limit", 2, -1.7, 1.7, 0.05, delta);
        auto tip = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
        limit->set_gain(0.5);
        tip->set_gain(2.0);
        const TaskList tasks = with_limit ? TaskList{limit, tip} : TaskList{tip};
        return simulate_task_priority(model, q0, tasks, PriorityConfig{}, 0.01, 1000);
    };
    const PriorityTrajectory free = run(0.15, false);
    const PriorityTrajectory limited = run(0.15, true);
    CHECK(free.q.col(2).maxCoeff() > 1.85);
    CHECK(limited.q.col(2).maxCoeff() < 1.7);
    CHECK(limited.error_norm(1000, 1) < 1e-6);
    CHECK(switches(limited.active.col(0)) == 2);  // on once, off once
    // Without hysteresis the activation chatters.
    const PriorityTrajectory chatter = run(0.05, true);
    CHECK(switches(chatter.active.col(0)) > 20);
    CHECK(chatter.q.col(2).maxCoeff() < 1.7);
}

TEST_CASE("obstacle task keeps the joint-3 point out of the obstacle", "[inequality]") {
    // Section 8.5: the obstacle sits on the path the joint-3 point takes without it.
    const SerialArmModel model(robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(-0.4, 1.2, 0.8);
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 0.2, 0.75, 0.0;
    const Eigen::Vector3d c(0.5205, 0.4496, 0.0);
    const double radius = 0.12;
    auto run = [&](bool with_obstacle) {
        auto obstacle = std::make_shared<ObstacleTask>("obstacle", 2, c, radius, 0.03, 0.08, true);
        auto tip = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
        obstacle->set_gain(0.5);
        tip->set_gain(2.0);
        const TaskList tasks = with_obstacle ? TaskList{obstacle, tip} : TaskList{tip};
        return simulate_task_priority(model, q0, tasks, PriorityConfig{}, 0.01, 800);
    };
    const PriorityTrajectory free = run(false), avoided = run(true);
    CHECK(max_distance_violation(model, free.q, 2, c.head<2>(), radius) > 0.1);
    CHECK(max_distance_violation(model, avoided.q, 2, c.head<2>(), radius) < 0.0);
    CHECK(avoided.error_norm(800, 1) < 1e-6);
}

TEST_CASE("an inactive inequality task does not change the motion", "[inequality]") {
    const SerialArmModel model(robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(0.3, 0.7, 0.6);
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 0.4, 0.8, 0.0;
    auto tip_a = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
    auto tip_b = std::make_shared<FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
    auto far = std::make_shared<ObstacleTask>("far", 2, Eigen::Vector3d(5.0, 5.0, 0.0), 0.1, 0.05, 0.1, true);
    const PriorityTrajectory a = simulate_task_priority(model, q0, {tip_a}, PriorityConfig{}, 0.01, 300);
    const PriorityTrajectory b = simulate_task_priority(model, q0, {far, tip_b}, PriorityConfig{}, 0.01, 300);
    CHECK(near(a.q, b.q, 1e-14));
    CHECK(b.active.col(0).maxCoeff() == 0.0);
}
