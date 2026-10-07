// Python module manipulator_control._core. One bind_* function per part of the C++ library.

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_transforms(py::module_& m);
void bind_kinematics(py::module_& m);
void bind_jacobian(py::module_& m);
void bind_singularities(py::module_& m);
void bind_resolved_rate(py::module_& m);

PYBIND11_MODULE(_core, m) {
    m.doc() = "C++ core of manipulator_control (kinematics and kinematic control of manipulators)";
    bind_transforms(m);
    bind_kinematics(m);
    bind_jacobian(m);
    bind_singularities(m);
    bind_resolved_rate(m);
}
