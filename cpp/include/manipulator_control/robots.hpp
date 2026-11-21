#pragma once
/// @file robots.hpp
/// Example arms used in the tests, examples and notebooks. They are data, not part of any algorithm.

#include <vector>

#include "manipulator_control/kinematics.hpp"

namespace manipulator_control::robots {

/// Planar arm with revolute joints about z and the given link lengths (DH: a_i = length, all else zero).
SerialChain planar(const std::vector<double>& lengths);

/// PUMA 560 in standard DH parameters, as tabulated by Corke (Robotics, Vision and Control). Six revolute
/// joints, spherical wrist. Units: metres.
SerialChain puma560();

/// Stanford-type arm: two revolute joints, one prismatic joint, spherical wrist (Siciliano et al.,
/// sec. 2.9.6). Link offsets d2 = 0.154 m and d6 = 0.263 m.
SerialChain stanford();

/// Anthropomorphic (elbow) arm with three revolute joints (Siciliano et al., sec. 2.9.3): a vertical waist
/// joint at height @p d1, then shoulder and elbow about horizontal axes with link lengths @p a2 and @p a3.
SerialChain anthropomorphic(double d1 = 0.1, double a2 = 0.3, double a3 = 0.25);

}  // namespace manipulator_control::robots
