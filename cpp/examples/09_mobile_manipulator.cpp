// Chapter 9 example: a unicycle base with an anthropomorphic arm reaches a point 2.2 m away, with the base
// weight given on the command line. Usage: 09_mobile_manipulator [base_weight]

#include <cstdlib>
#include <iostream>
#include <memory>

#include "manipulator_control/inequality_tasks.hpp"
#include "manipulator_control/mobile_manipulator.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double base_weight = argc > 1 ? std::atof(argv[1]) : 10.0;
    Eigen::Matrix4d mount = Eigen::Matrix4d::Identity();
    mount.block<3, 1>(0, 3) << 0.1, 0.0, 0.2;
    const mc::MobileManipulator robot(mc::robots::anthropomorphic(), mount);
    Eigen::VectorXd q0(6);
    q0 << 0.0, 0.0, 0.0, 0.0, 0.6, -1.2;
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 2.0, 1.0, 0.3;

    auto ee = std::make_shared<mc::FrameTask>("end effector", -1, std::vector<int>{0, 1, 2}, target);
    auto elbow = std::make_shared<mc::JointLimitTask>("elbow limit", 4, -2.5, -0.5, 0.05, 0.15);
    elbow->set_gain(0.5);
    mc::PriorityConfig config;
    config.inverse = mc::InverseMethod::DampedLeastSquares;
    config.damping = 0.05;
    config.weights = Eigen::VectorXd::Ones(5);
    config.weights.head(2) *= base_weight;
    const mc::PriorityTrajectory r = mc::simulate_task_priority(robot, q0, {elbow, ee}, config, 0.01, 1500);
    std::cout << "base weight " << base_weight << "\n"
              << "final configuration (x, y, psi, q_arm): " << r.q.bottomRows(1) << "\n"
              << "end-effector error: " << r.error_norm(1500, 1) << " m\n";
    return 0;
}
