// Bindings for chapter 8: set-based inequality tasks.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "manipulator_control/inequality_tasks.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_inequality_tasks(py::module_& m) {
    m.def("next_activation", &mc::next_activation, py::arg("state"), py::arg("sigma"), py::arg("sigma_min"),
          py::arg("sigma_max"), py::arg("alpha"), py::arg("delta"),
          "Activation state machine of a set-based task (eq. 8.3): 0, +1 (lower bound), -1 (upper bound).");

    py::class_<mc::SetBasedTask, mc::Task, std::shared_ptr<mc::SetBasedTask>>(
        m, "SetBasedTask", "Scalar task kept inside [sigma_min, sigma_max] (eqs. 8.2-8.4).")
        .def_property_readonly("activation", &mc::SetBasedTask::activation)
        .def_property_readonly("value", &mc::SetBasedTask::value)
        .def_property_readonly("sigma_min", &mc::SetBasedTask::sigma_min)
        .def_property_readonly("sigma_max", &mc::SetBasedTask::sigma_max)
        .def_property_readonly("alpha", &mc::SetBasedTask::alpha)
        .def_property_readonly("delta", &mc::SetBasedTask::delta)
        .def("reset", &mc::SetBasedTask::reset);

    py::class_<mc::JointLimitTask, mc::SetBasedTask, std::shared_ptr<mc::JointLimitTask>>(
        m, "JointLimitTask", "q_min <= q_j <= q_max (eq. 8.5).")
        .def(py::init<std::string, int, double, double, double, double>(), py::arg("name"),
             py::arg("control_index"), py::arg("q_min"), py::arg("q_max"), py::arg("alpha"), py::arg("delta"))
        .def_property_readonly("control_index", &mc::JointLimitTask::control_index);

    py::class_<mc::ObstacleTask, mc::SetBasedTask, std::shared_ptr<mc::ObstacleTask>>(
        m, "ObstacleTask", "Distance of a frame origin from a point obstacle, at least `radius` (eq. 8.6).")
        .def(py::init<std::string, int, const Eigen::Vector3d&, double, double, double, bool>(), py::arg("name"),
             py::arg("frame"), py::arg("center"), py::arg("radius"), py::arg("alpha"), py::arg("delta"),
             py::arg("planar") = false)
        .def_property("center", &mc::ObstacleTask::center, &mc::ObstacleTask::set_center)
        .def_property_readonly("frame", &mc::ObstacleTask::frame);
}
