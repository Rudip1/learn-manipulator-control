// Chapter 2 example: the PUMA 560 end-effector pose from DH parameters and from the product of exponentials.
// Usage: 02_forward_kinematics [q1 ... q6]

#include <cstdlib>
#include <iostream>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/robots.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const mc::SerialChain arm = mc::robots::puma560();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(arm.dof());
    for (int i = 0; i < arm.dof() && i + 1 < argc; ++i) q(i) = std::atof(argv[i + 1]);

    const mc::PoeChain poe = mc::to_poe(arm);
    Eigen::IOFormat fmt(6, 0, "  ", "\n", "  [", "]");
    std::cout << "q = " << q.transpose().format(fmt) << "\n";
    std::cout << "DH:\n" << arm.end_effector(q).format(fmt) << "\n";
    std::cout << "product of exponentials:\n" << mc::poe_space(poe, q).format(fmt) << "\n";
    std::cout << "screw axes (v, w):\n";
    for (const auto& S : poe.screws) std::cout << S.transpose().format(fmt) << "\n";
    return 0;
}
