// Chapter 4 example: resolved-rate control of a planar two-link arm with the three inverses.
// Usage: 04_resolved_rate [x_goal y_goal [gain]]

#include <cstdlib>
#include <iostream>

#include "manipulator_control/resolved_rate.hpp"
#include "manipulator_control/robots.hpp"

int main(int argc, char** argv) {
    namespace mc = manipulator_control;
    const double x = argc > 2 ? std::atof(argv[1]) : 0.0, y = argc > 2 ? std::atof(argv[2]) : 1.0;
    const mc::SerialChain arm = mc::robots::planar({0.75, 0.5});
    mc::ResolvedRateConfig cfg;
    cfg.space = mc::TaskSpace::PlanarPosition;
    cfg.gain = argc > 3 ? std::atof(argv[3]) : 1.0;
    cfg.dt = 1.0 / 60.0;
    cfg.steps = 600;

    std::cout << "goal (" << x << ", " << y << "), gain " << cfg.gain << ", dt " << cfg.dt << " s\n";
    for (auto [name, method] : {std::pair{"transpose    ", mc::InverseMethod::Transpose},
                                std::pair{"pseudoinverse", mc::InverseMethod::Pseudoinverse},
                                std::pair{"DLS          ", mc::InverseMethod::DampedLeastSquares}}) {
        cfg.method = method;
        const mc::Trajectory traj =
            mc::simulate_resolved_rate(arm, Eigen::Vector2d(0.2, 0.5), mc::translation({x, y, 0.0}), cfg);
        std::cout << name << "  |error| at t = 0, 2, 5, 10 s: " << traj.error_norm(0) << "  "
                  << traj.error_norm(120) << "  " << traj.error_norm(300) << "  " << traj.error_norm(600)
                  << "\n";
    }
    return 0;
}
