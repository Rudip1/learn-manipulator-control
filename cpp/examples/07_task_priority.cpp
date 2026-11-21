// Chapter 7 example: hold the tip of a three-link arm and drive joint 1 into the algorithmic singularity,
// with the pseudoinverse, the DLS inverse and the singularity-robust method. Usage: 07_task_priority [lambda]

#include <cstdlib>
#include <iostream>
#include <memory>

#include "manipulator_control/robots.hpp"
#include "manipulator_control/task_priority.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double lambda = argc > 1 ? std::atof(argv[1]) : 0.05;
    const mc::SerialArmModel model(mc::robots::planar({0.5, 0.4, 0.3}));
    const Eigen::Vector3d q0(0.3, 0.7, 0.6);
    struct Run {
        const char* name;
        mc::PriorityMethod method;
        mc::InverseMethod inverse;
    };
    std::cout << "method                 peak |qdot|   max tip error   final q1\n";
    for (const Run& run : {Run{"recursive, pinv     ", mc::PriorityMethod::Recursive, mc::InverseMethod::Pseudoinverse},
                           Run{"recursive, DLS      ", mc::PriorityMethod::Recursive,
                               mc::InverseMethod::DampedLeastSquares},
                           Run{"singularity-robust  ", mc::PriorityMethod::SingularityRobust,
                               mc::InverseMethod::Pseudoinverse}}) {
        auto tip = std::make_shared<mc::FrameTask>("tip", -1, std::vector<int>{0, 1}, model.frame_pose(q0, -1));
        auto joint = std::make_shared<mc::JointTask>("q1", 0, 1.7);
        tip->set_gain(2.0);
        joint->set_gain(2.0);
        mc::PriorityConfig config;
        config.method = run.method;
        config.inverse = run.inverse;
        config.damping = lambda;
        const mc::PriorityTrajectory r = mc::simulate_task_priority(model, q0, {tip, joint}, config, 0.01, 800);
        std::cout << run.name << "   " << r.u.cwiseAbs().maxCoeff() << "\t" << r.error_norm.col(0).maxCoeff()
                  << "\t" << r.q(800, 0) << "\n";
    }
    return 0;
}
