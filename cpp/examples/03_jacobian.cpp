// Chapter 3 example: geometric Jacobian of the PUMA 560 and its finite-difference check.
// Usage: 03_jacobian [q1 ... q6]

#include <cstdlib>
#include <iostream>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/robots.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const mc::SerialChain arm = mc::robots::puma560();
    Eigen::VectorXd q(6);
    q << 0.3, 0.8, -0.5, 0.2, 0.9, -0.4;
    for (int i = 0; i < arm.dof() && i + 1 < argc; ++i) q(i) = std::atof(argv[i + 1]);

    const Eigen::MatrixXd J = mc::geometric_jacobian(arm, q);
    const Eigen::MatrixXd J_num = mc::numerical_jacobian(arm, q);
    Eigen::IOFormat fmt(4, 0, "  ", "\n", "  [", "]");
    std::cout << "q = " << q.transpose().format(fmt) << "\n";
    std::cout << "geometric Jacobian (rows v, w):\n" << J.format(fmt) << "\n";
    std::cout << "max |J - J_finite_differences| = " << (J - J_num).cwiseAbs().maxCoeff() << "\n";
    return 0;
}
