// Bindings for chapter 6: null-space projectors, weighted pseudoinverse, gradient projection.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "manipulator_control/redundancy.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_redundancy(py::module_& m) {
    m.def("null_space_projector",
          py::overload_cast<const Eigen::MatrixXd&, double>(&mc::null_space_projector), py::arg("J"),
          py::arg("tol") = 1e-10, "N = I - J^+ J (eq. 6.3).");
    m.def("null_space_projector_of",
          py::overload_cast<const Eigen::MatrixXd&, const Eigen::MatrixXd&>(&mc::null_space_projector),
          py::arg("J"), py::arg("J_inverse"), "I - J^# J for a given generalised inverse J^#.");
    m.def("weighted_pseudoinverse", &mc::weighted_pseudoinverse, py::arg("J"), py::arg("W"),
          py::arg("tol") = 1e-10, "W^-1 J^T (J W^-1 J^T)^-1, minimum weighted-norm inverse (eq. 6.6).");
    m.def("joint_centering_cost", &mc::joint_centering_cost, py::arg("q"), py::arg("q_min"), py::arg("q_max"),
          "H(q) = 1/(2n) sum ((q_i - qbar_i) / range_i)^2 (eq. 6.9).");
    m.def("joint_centering_gradient", &mc::joint_centering_gradient, py::arg("q"), py::arg("q_min"),
          py::arg("q_max"), "Gradient of H (eq. 6.10).");
    m.def("task_manipulability", &mc::task_manipulability, py::arg("chain"), py::arg("q"), py::arg("space"),
          "sqrt(det(J J^T)) of the task Jacobian.");
    m.def("manipulability_gradient", &mc::manipulability_gradient, py::arg("chain"), py::arg("q"),
          py::arg("space"), py::arg("h") = 1e-6, "Gradient of w by central differences (eq. 6.11).");

    py::enum_<mc::SecondaryObjective>(m, "SecondaryObjective")
        .value("None_", mc::SecondaryObjective::None)
        .value("JointCentering", mc::SecondaryObjective::JointCentering)
        .value("Manipulability", mc::SecondaryObjective::Manipulability);

    py::class_<mc::RedundancyConfig>(m, "RedundancyConfig")
        .def(py::init([](mc::SecondaryObjective objective, double k0, const Eigen::VectorXd& q_min,
                         const Eigen::VectorXd& q_max, bool project) {
                 mc::RedundancyConfig c;
                 c.objective = objective;
                 c.k0 = k0;
                 c.q_min = q_min;
                 c.q_max = q_max;
                 c.project = project;
                 return c;
             }),
             py::arg("objective") = mc::SecondaryObjective::None, py::arg("k0") = 1.0,
             py::arg("q_min") = Eigen::VectorXd(), py::arg("q_max") = Eigen::VectorXd(),
             py::arg("project") = true)
        .def_readwrite("objective", &mc::RedundancyConfig::objective)
        .def_readwrite("k0", &mc::RedundancyConfig::k0)
        .def_readwrite("q_min", &mc::RedundancyConfig::q_min)
        .def_readwrite("q_max", &mc::RedundancyConfig::q_max)
        .def_readwrite("project", &mc::RedundancyConfig::project);

    m.def("secondary_objective_value", &mc::secondary_objective_value, py::arg("chain"), py::arg("q"),
          py::arg("space"), py::arg("redundancy"), "H, w or 0 at q.");
    m.def("secondary_velocity", &mc::secondary_velocity, py::arg("chain"), py::arg("q"), py::arg("space"),
          py::arg("redundancy"), "z = -k0 grad H or +k0 grad w (eq. 6.8).");
    m.def("redundancy_step", &mc::redundancy_step, py::arg("chain"), py::arg("q"), py::arg("T_desired"),
          py::arg("feedforward"), py::arg("config"), py::arg("redundancy"),
          "qdot = J^# (K e + feedforward) + (I - J^# J) z (eq. 6.8).");

    py::class_<mc::RedundancyTrajectory>(m, "RedundancyTrajectory")
        .def_readonly("trajectory", &mc::RedundancyTrajectory::trajectory)
        .def_readonly("objective", &mc::RedundancyTrajectory::objective);
    m.def("simulate_redundancy", &mc::simulate_redundancy, py::arg("chain"), py::arg("q0"),
          py::arg("T_desired"), py::arg("config"), py::arg("redundancy"),
          "Gradient projection to a fixed pose (section 6.5).");
}
