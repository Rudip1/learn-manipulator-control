// Bindings for chapter 3: Jacobians.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "manipulator_control/jacobian.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_jacobian(py::module_& m) {
    m.def("geometric_jacobian",
          py::overload_cast<const mc::SerialChain&, const Eigen::VectorXd&>(&mc::geometric_jacobian),
          py::arg("chain"), py::arg("q"), "End-effector geometric Jacobian, rows (v, w) (eq. 3.3).");
    m.def("geometric_jacobian_from_frames",
          py::overload_cast<const std::vector<Eigen::Matrix4d>&, const std::vector<mc::JointType>&, int>(
              &mc::geometric_jacobian),
          py::arg("frames"), py::arg("types"), py::arg("k"),
          "Geometric Jacobian of the origin of frames[k] (eqs. 3.2-3.4).");
    m.def("link_jacobian", &mc::link_jacobian, py::arg("chain"), py::arg("q"), py::arg("k"),
          "Geometric Jacobian of DH frame k = 0..n, or of the end effector for k = n + 1 (eq. 3.4).");
    m.def("space_jacobian", &mc::space_jacobian, py::arg("chain"), py::arg("q"), "Space Jacobian (eq. 3.5).");
    m.def("body_jacobian", &mc::body_jacobian, py::arg("chain"), py::arg("q"), "Body Jacobian (eq. 3.6).");
    m.def("rpy_rate_matrix", &mc::rpy_rate_matrix, py::arg("rpy"),
          "T(rpy) with w = T(rpy) d(rpy)/dt; det = cos(pitch) (eq. 3.8).");
    m.def("pose_rpy", &mc::pose_rpy, py::arg("T"), "(x, y, z, roll, pitch, yaw) of a transform.");
    m.def("analytic_jacobian", &mc::analytic_jacobian, py::arg("chain"), py::arg("q"),
          "Analytic Jacobian of (p, roll, pitch, yaw) (eq. 3.9).");
    m.def("numerical_jacobian", &mc::numerical_jacobian, py::arg("chain"), py::arg("q"), py::arg("h") = 1e-6,
          py::arg("k") = -1, "Geometric Jacobian of frame k by central differences (eq. 3.10).");
    m.def("numerical_analytic_jacobian", &mc::numerical_analytic_jacobian, py::arg("chain"), py::arg("q"),
          py::arg("h") = 1e-6, "Analytic Jacobian by central differences (eq. 3.10).");
}
