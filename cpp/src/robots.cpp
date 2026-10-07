#include "manipulator_control/robots.hpp"

namespace manipulator_control::robots {

namespace {
constexpr double kHalfPi = 1.57079632679489661923;
}

SerialChain planar(const std::vector<double>& lengths) {
    std::vector<DHLink> links;
    for (double l : lengths) links.push_back({l, 0.0, 0.0, 0.0, JointType::Revolute});
    return SerialChain(links);
}

SerialChain puma560() {
    //        a        alpha     d        theta
    return SerialChain({{0.0, kHalfPi, 0.0, 0.0},
                        {0.4318, 0.0, 0.0, 0.0},
                        {0.0203, -kHalfPi, 0.15005, 0.0},
                        {0.0, kHalfPi, 0.4318, 0.0},
                        {0.0, -kHalfPi, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0}});
}

SerialChain stanford() {
    //        a    alpha     d      theta  type
    return SerialChain({{0.0, -kHalfPi, 0.0, 0.0, JointType::Revolute},
                        {0.0, kHalfPi, 0.154, 0.0, JointType::Revolute},
                        {0.0, 0.0, 0.0, 0.0, JointType::Prismatic},
                        {0.0, -kHalfPi, 0.0, 0.0, JointType::Revolute},
                        {0.0, kHalfPi, 0.0, 0.0, JointType::Revolute},
                        {0.0, 0.0, 0.263, 0.0, JointType::Revolute}});
}

}  // namespace manipulator_control::robots
