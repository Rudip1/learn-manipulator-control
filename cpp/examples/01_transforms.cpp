// Chapter 1 example: a screw motion and its logarithm.
// Usage: 01_transforms [angle_rad]

#include <cstdlib>
#include <iostream>

#include "manipulator_control/transforms.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double angle = argc > 1 ? std::atof(argv[1]) : 1.5707963267948966;

    // Rotation about the vertical line through (1, 0, 0) with zero pitch (worked example of section 1.3).
    const mc::Vector6d S = mc::screw_axis({1.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ());
    const Eigen::Matrix4d T = mc::exp_se3(S * angle);

    Eigen::IOFormat fmt(6, 0, "  ", "\n", "  [", "]");
    std::cout << "screw axis S = " << S.transpose().format(fmt) << "\n";
    std::cout << "T = exp([S] * " << angle << ") =\n" << T.format(fmt) << "\n";
    std::cout << "log(T) = " << mc::log_se3(T).transpose().format(fmt) << "\n";
    std::cout << "the point (1,0,0) maps to "
              << (T * Eigen::Vector4d(1, 0, 0, 1)).head<3>().transpose().format(fmt) << "\n";
    return 0;
}
