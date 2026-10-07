// Chapter 5 example: distance to the PUMA 560 wrist singularity, and what the pseudoinverse and the DLS
// inverse do with a unit task velocity there. Usage: 05_singularities [lambda]

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "manipulator_control/jacobian.hpp"
#include "manipulator_control/resolved_rate.hpp"
#include "manipulator_control/robots.hpp"
#include "manipulator_control/singularities.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double lambda = argc > 1 ? std::atof(argv[1]) : 0.05;
    const mc::SerialChain arm = mc::robots::puma560();
    std::cout << "q5      w          s_min      |J^+ u_min|  |J_dls u_min|\n";
    for (double q5 : {0.5, 0.2, 0.1, 0.05, 0.01, 0.001}) {
        Eigen::VectorXd q(6);
        q << 0.0, 0.6, -0.3, 0.5, q5, 0.0;
        const Eigen::MatrixXd J = mc::geometric_jacobian(arm, q);
        const mc::Ellipsoid e = mc::manipulability_ellipsoid(J);
        const Eigen::VectorXd u = e.axes.col(5);  // the task direction being lost
        std::cout << q5 << "\t" << mc::manipulability(J) << "\t" << e.radii(5) << "\t"
                  << (mc::pseudoinverse(J) * u).norm() << "\t" << (mc::dls_inverse(J, lambda) * u).norm()
                  << "\n";
    }
    std::cout << "DLS bound 1/(2 lambda) = " << 1.0 / (2.0 * lambda) << "\n";
    return 0;
}
