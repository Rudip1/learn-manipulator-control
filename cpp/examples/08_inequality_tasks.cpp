// Chapter 8 example: a joint limit above an end-effector task, with and without hysteresis.
// Usage: 08_inequality_tasks [delta]

#include <cstdlib>
#include <iostream>
#include <memory>

#include "manipulator_control/inequality_tasks.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double delta = argc > 1 ? std::atof(argv[1]) : 0.15;
    const mc::SerialArmModel model(mc::robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(1.2, -1.5, 1.6);
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 1>(0, 3) << 0.6, 0.5, 0.0;
    std::cout << "delta   max q3 [rad]   switches   final tip error [m]\n";
    for (double d : {0.05, delta}) {
        auto limit = std::make_shared<mc::JointLimitTask>("q3 limit", 2, -1.7, 1.7, 0.05, d);
        auto tip = std::make_shared<mc::FrameTask>("tip", -1, std::vector<int>{0, 1}, target);
        limit->set_gain(0.5);
        tip->set_gain(2.0);
        const mc::PriorityTrajectory r =
            mc::simulate_task_priority(model, q0, {limit, tip}, mc::PriorityConfig{}, 0.01, 1000);
        int switches = 0;
        for (Eigen::Index k = 1; k < r.active.rows(); ++k) switches += r.active(k, 0) != r.active(k - 1, 0);
        std::cout << d << "\t" << r.q.col(2).maxCoeff() << "\t" << switches << "\t" << r.error_norm(1000, 1) << "\n";
    }
    return 0;
}
