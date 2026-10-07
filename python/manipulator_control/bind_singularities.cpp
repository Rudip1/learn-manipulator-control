// Bindings for chapter 5: singular values, manipulability, damping schedules.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "manipulator_control/singularities.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_singularities(py::module_& m) {
    m.def("singular_values", &mc::singular_values, py::arg("J"), "Singular values, largest first.");
    m.def("manipulability", &mc::manipulability, py::arg("J"), "sqrt(det(J J^T)) (eq. 5.2).");
    m.def("condition_number", &mc::condition_number, py::arg("J"), "s_1 / s_m.");

    py::class_<mc::Ellipsoid>(m, "Ellipsoid", "Velocity ellipsoid: axes u_i (columns) and radii s_i.")
        .def_readonly("axes", &mc::Ellipsoid::axes)
        .def_readonly("radii", &mc::Ellipsoid::radii);
    m.def("manipulability_ellipsoid", &mc::manipulability_ellipsoid, py::arg("J"),
          "Principal axes and radii of {J qdot : |qdot| <= 1} (eq. 5.1).");

    py::enum_<mc::DampingSchedule>(m, "DampingSchedule")
        .value("Constant", mc::DampingSchedule::Constant)
        .value("Manipulability", mc::DampingSchedule::Manipulability)
        .value("MinSingularValue", mc::DampingSchedule::MinSingularValue);
    m.def("damping_factor", &mc::damping_factor, py::arg("J"), py::arg("schedule"), py::arg("lambda_max"),
          py::arg("threshold"), "DLS damping for a schedule (eqs. 5.8, 5.9).");
}
