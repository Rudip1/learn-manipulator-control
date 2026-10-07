// Bindings for chapter 1: rotations, transforms, twists.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "manipulator_control/transforms.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_transforms(py::module_& m) {
    m.def("skew", &mc::skew, py::arg("w"), "Skew-symmetric matrix [w]x with [w]x u = w x u (eq. 1.3).");
    m.def("unskew", &mc::unskew, py::arg("S"), "Vector of the skew-symmetric part of S (inverse of skew).");
    m.def("rot_x", &mc::rot_x, py::arg("angle"), "Rotation about x (eq. 1.2).");
    m.def("rot_y", &mc::rot_y, py::arg("angle"), "Rotation about y (eq. 1.2).");
    m.def("rot_z", &mc::rot_z, py::arg("angle"), "Rotation about z (eq. 1.2).");
    m.def("is_rotation", &mc::is_rotation, py::arg("R"), py::arg("tol") = 1e-9,
          "True if R is orthonormal with determinant +1.");
    m.def("exp_so3", &mc::exp_so3, py::arg("phi"),
          "Rodrigues' formula: rotation by |phi| about phi (eq. 1.5).");
    m.def("log_so3", &mc::log_so3, py::arg("R"), "Exponential coordinates of R, |phi| in [0, pi] (eq. 1.6).");
    m.def("rotation_from_rpy", &mc::rotation_from_rpy, py::arg("rpy"),
          "R = Rz(yaw) Ry(pitch) Rx(roll) from (roll, pitch, yaw) (eq. 1.7).");
    m.def("rpy_from_rotation", &mc::rpy_from_rotation, py::arg("R"),
          "(roll, pitch, yaw) of R; roll = 0 at the singularity pitch = +-pi/2.");
    m.def("make_transform", &mc::make_transform, py::arg("R"), py::arg("p"),
          "Homogeneous transform from rotation and translation (eq. 1.8).");
    m.def("translation", &mc::translation, py::arg("p"), "Pure translation.");
    m.def("inverse_transform", &mc::inverse_transform, py::arg("T"),
          "Inverse of a transform via R^T (eq. 1.9).");
    m.def("twist_hat", &mc::twist_hat, py::arg("V"), "4x4 se(3) matrix of a twist V = (v, w) (eq. 1.10).");
    m.def("twist_vee", &mc::twist_vee, py::arg("V_hat"), "Twist (v, w) of a 4x4 se(3) matrix.");
    m.def("adjoint", &mc::adjoint, py::arg("T"), "6x6 adjoint of T for (v, w) twists (eq. 1.11).");
    m.def("screw_axis", &mc::screw_axis, py::arg("q"), py::arg("w_hat"), py::arg("h") = 0.0,
          "Screw axis through q along w_hat with pitch h (eq. 1.12).");
    m.def("exp_se3", &mc::exp_se3, py::arg("xi"), "Exponential map on SE(3) of xi = S*theta (eq. 1.13).");
    m.def("log_se3", &mc::log_se3, py::arg("T"), "Exponential coordinates of T (eq. 1.14).");
}
