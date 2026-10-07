// Bindings for chapter 4: generalised inverses and resolved-rate control.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>

#include "manipulator_control/resolved_rate.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_resolved_rate(py::module_& m) {
    m.def("pseudoinverse", &mc::pseudoinverse, py::arg("J"), py::arg("tol") = 1e-10,
          "Moore-Penrose pseudoinverse by SVD (eq. 4.4).");
    m.def("dls_inverse", &mc::dls_inverse, py::arg("J"), py::arg("damping"),
          "Damped least-squares inverse J^T (J J^T + lambda^2 I)^-1 (eq. 4.6).");

    py::enum_<mc::InverseMethod>(m, "InverseMethod")
        .value("Transpose", mc::InverseMethod::Transpose)
        .value("Pseudoinverse", mc::InverseMethod::Pseudoinverse)
        .value("DampedLeastSquares", mc::InverseMethod::DampedLeastSquares);
    m.def("generalized_inverse", &mc::generalized_inverse, py::arg("J"), py::arg("method"),
          py::arg("damping") = 0.1, "J^T, J^+ or the DLS inverse.");

    py::enum_<mc::TaskSpace>(m, "TaskSpace")
        .value("PlanarPosition", mc::TaskSpace::PlanarPosition)
        .value("Position", mc::TaskSpace::Position)
        .value("Pose", mc::TaskSpace::Pose);
    m.def("task_dimension", &mc::task_dimension, py::arg("space"));
    m.def("orientation_error", &mc::orientation_error, py::arg("R_desired"), py::arg("R"),
          "log(R_d R^T), axis times angle in the base frame (eq. 4.10).");
    m.def("task_error", &mc::task_error, py::arg("space"), py::arg("T_desired"), py::arg("T"),
          "sigma_d - sigma for a task space (eqs. 4.2, 4.10).");
    m.def("task_jacobian", &mc::task_jacobian, py::arg("space"), py::arg("J"),
          "Rows of the geometric Jacobian used by a task space.");

    py::class_<mc::ResolvedRateConfig>(m, "ResolvedRateConfig")
        .def(py::init([](mc::InverseMethod method, mc::TaskSpace space, double gain, double damping,
                         double dt, int steps, double max_joint_speed, mc::DampingSchedule damping_schedule,
                         double damping_threshold) {
                 mc::ResolvedRateConfig c;
                 c.method = method;
                 c.space = space;
                 c.gain = gain;
                 c.damping = damping;
                 c.dt = dt;
                 c.steps = steps;
                 c.max_joint_speed = max_joint_speed;
                 c.damping_schedule = damping_schedule;
                 c.damping_threshold = damping_threshold;
                 return c;
             }),
             py::arg("method") = mc::InverseMethod::Pseudoinverse, py::arg("space") = mc::TaskSpace::Position,
             py::arg("gain") = 1.0, py::arg("damping") = 0.1, py::arg("dt") = 0.01, py::arg("steps") = 1000,
             py::arg("max_joint_speed") = std::numeric_limits<double>::infinity(),
             py::arg("damping_schedule") = mc::DampingSchedule::Constant, py::arg("damping_threshold") = 0.05)
        .def_readwrite("method", &mc::ResolvedRateConfig::method)
        .def_readwrite("space", &mc::ResolvedRateConfig::space)
        .def_readwrite("gain", &mc::ResolvedRateConfig::gain)
        .def_readwrite("damping", &mc::ResolvedRateConfig::damping)
        .def_readwrite("dt", &mc::ResolvedRateConfig::dt)
        .def_readwrite("steps", &mc::ResolvedRateConfig::steps)
        .def_readwrite("max_joint_speed", &mc::ResolvedRateConfig::max_joint_speed)
        .def_readwrite("damping_schedule", &mc::ResolvedRateConfig::damping_schedule)
        .def_readwrite("damping_threshold", &mc::ResolvedRateConfig::damping_threshold);

    py::class_<mc::Trajectory>(m, "Trajectory", "Logged simulation: arrays with one row per sample.")
        .def_readonly("t", &mc::Trajectory::t)
        .def_readonly("q", &mc::Trajectory::q)
        .def_readonly("qdot", &mc::Trajectory::qdot)
        .def_readonly("ee_position", &mc::Trajectory::ee_position)
        .def_readonly("error", &mc::Trajectory::error)
        .def_readonly("error_norm", &mc::Trajectory::error_norm);

    m.def("resolved_rate_step", &mc::resolved_rate_step, py::arg("chain"), py::arg("q"), py::arg("T_desired"),
          py::arg("feedforward"), py::arg("config"), "qdot = J^# (K e + feedforward) (eq. 4.3).");
    m.def("simulate_resolved_rate", &mc::simulate_resolved_rate, py::arg("chain"), py::arg("q0"),
          py::arg("T_desired"), py::arg("config"),
          "Resolved-rate control to a fixed pose (sections 4.6, 4.8).");
    m.def("simulate_position_tracking", &mc::simulate_position_tracking, py::arg("chain"), py::arg("q0"),
          py::arg("positions"), py::arg("velocities"), py::arg("config"),
          "Track a sampled position reference with feedforward velocities (section 4.5).");
    m.def("simulate_pose_tracking", &mc::simulate_pose_tracking, py::arg("chain"), py::arg("q0"),
          py::arg("poses"), py::arg("twists"), py::arg("config"),
          "Track a sampled pose reference with feedforward twists (v, w) in the base frame (section 4.5).");
}
