// Chapter 6 example: the same target reached by the minimum-norm solution and by gradient projection with
// joint centering, on a planar three-link arm. Usage: 06_redundancy [k0]

#include <cstdlib>
#include <iostream>

#include "manipulator_control/redundancy.hpp"
#include "manipulator_control/robots.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double k0 = argc > 1 ? std::atof(argv[1]) : 20.0;
    const mc::SerialChain arm = mc::robots::planar({0.5, 0.4, 0.3});
    Eigen::VectorXd q0(3), q_min(3), q_max(3);
    q0 << 1.2, -1.5, 1.6;
    q_min << -1.5707963267948966, -2.0, -2.0;
    q_max = -q_min;
    Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
    T_d.block<3, 1>(0, 3) << 0.6, 0.5, 0.0;
    mc::ResolvedRateConfig config;
    config.space = mc::TaskSpace::PlanarPosition;
    config.gain = 2.0;
    config.dt = 0.01;
    config.steps = 1000;
    mc::RedundancyConfig redundancy;
    redundancy.objective = mc::SecondaryObjective::JointCentering;
    redundancy.q_min = q_min;
    redundancy.q_max = q_max;
    std::cout << "k0      final error [m]   H(q_end)   q_end\n";
    for (double k : {0.0, k0}) {
        redundancy.k0 = k;
        const mc::RedundancyTrajectory r = mc::simulate_redundancy(arm, q0, T_d, config, redundancy);
        std::cout << k << "\t" << r.trajectory.error_norm(config.steps) << "\t" << r.objective(config.steps)
                  << "\t" << r.trajectory.q.bottomRows(1) << "\n";
    }
    return 0;
}
