// Bindings for chapter 7: robot models, tasks and the task-priority solver.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>

#include "manipulator_control/mobile_manipulator.hpp"
#include "manipulator_control/robot_model.hpp"
#include "manipulator_control/task_priority.hpp"
#include "manipulator_control/tasks.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_task_priority(py::module_& m) {
    py::class_<mc::RobotModel, std::shared_ptr<mc::RobotModel>>(
        m, "RobotModel", "Kinematic interface: configuration q, commanded velocity u, frames and Jacobians.")
        .def("state_size", &mc::RobotModel::state_size)
        .def("control_size", &mc::RobotModel::control_size)
        .def("frame_pose", &mc::RobotModel::frame_pose, py::arg("q"), py::arg("k") = -1)
        .def("frame_jacobian", &mc::RobotModel::frame_jacobian, py::arg("q"), py::arg("k") = -1)
        .def("state_index", &mc::RobotModel::state_index, py::arg("j"))
        .def("integrate", &mc::RobotModel::integrate, py::arg("q"), py::arg("u"), py::arg("dt"));

    py::class_<mc::SerialArmModel, mc::RobotModel, std::shared_ptr<mc::SerialArmModel>>(
        m, "SerialArmModel", "Fixed-base arm: u = qdot, frames are the DH frames, -1 the end effector.")
        .def(py::init<mc::SerialChain>(), py::arg("chain"))
        .def_property_readonly("chain", &mc::SerialArmModel::chain);

    m.def("base_transform", &mc::base_transform, py::arg("eta"), "Pose (x, y, psi) as a 4x4 transform (eq. 9.3).");
    m.def("integrate_unicycle", &mc::integrate_unicycle, py::arg("eta"), py::arg("omega"), py::arg("v"),
          py::arg("dt"), "Exact unicycle step: an arc, or a segment for omega = 0 (eq. 9.6).");
    py::class_<mc::MobileManipulator, mc::RobotModel, std::shared_ptr<mc::MobileManipulator>>(
        m, "MobileManipulator",
        "Unicycle base + arm: q = (x, y, psi, q_arm), u = (omega, v, qdot_arm); frames -1 EE, 0 base, "
        "1..n+1 arm DH frames 0..n (eqs. 9.3-9.6).")
        .def(py::init<mc::SerialChain, const Eigen::Matrix4d&>(), py::arg("arm"), py::arg("mount"))
        .def_property_readonly("arm", &mc::MobileManipulator::arm)
        .def_property_readonly("mount", &mc::MobileManipulator::mount)
        .def_property_readonly("arm_dof", &mc::MobileManipulator::arm_dof);

    py::class_<mc::Task, std::shared_ptr<mc::Task>>(m, "Task", "A task: Jacobian, error, gain, feedforward.")
        .def_property_readonly("name", &mc::Task::name)
        .def_property_readonly("dimension", &mc::Task::dimension)
        .def("update", &mc::Task::update, py::arg("model"), py::arg("q"))
        .def("is_active", &mc::Task::is_active)
        .def_property_readonly("jacobian", &mc::Task::jacobian)
        .def_property_readonly("error", &mc::Task::error)
        .def("velocity", &mc::Task::velocity, "v = feedforward + K error (eq. 7.2).")
        .def("set_gain", py::overload_cast<double>(&mc::Task::set_gain), py::arg("k"))
        .def("set_gain_matrix", py::overload_cast<const Eigen::MatrixXd&>(&mc::Task::set_gain), py::arg("K"))
        .def_property_readonly("gain", &mc::Task::gain)
        .def("set_feedforward", &mc::Task::set_feedforward, py::arg("feedforward"));

    py::class_<mc::FrameTask, mc::Task, std::shared_ptr<mc::FrameTask>>(
        m, "FrameTask", "Components (x, y, z, rx, ry, rz) of a frame's pose error.")
        .def(py::init<std::string, int, std::vector<int>, const Eigen::Matrix4d&>(), py::arg("name"),
             py::arg("frame"), py::arg("components"), py::arg("desired"))
        .def_property("desired", &mc::FrameTask::desired, &mc::FrameTask::set_desired)
        .def_property_readonly("frame", &mc::FrameTask::frame)
        .def_property_readonly("components", &mc::FrameTask::components);

    py::class_<mc::JointTask, mc::Task, std::shared_ptr<mc::JointTask>>(m, "JointTask",
                                                                        "One joint to a desired position.")
        .def(py::init<std::string, int, double>(), py::arg("name"), py::arg("control_index"), py::arg("desired"))
        .def_property("desired", &mc::JointTask::desired, &mc::JointTask::set_desired)
        .def_property_readonly("control_index", &mc::JointTask::control_index);

    py::enum_<mc::PriorityMethod>(m, "PriorityMethod")
        .value("Recursive", mc::PriorityMethod::Recursive)
        .value("SingularityRobust", mc::PriorityMethod::SingularityRobust);

    py::class_<mc::PriorityConfig>(m, "PriorityConfig")
        .def(py::init([](mc::PriorityMethod method, mc::InverseMethod inverse, double damping, double max_speed,
                         const Eigen::VectorXd& weights) {
                 mc::PriorityConfig c;
                 c.method = method;
                 c.inverse = inverse;
                 c.damping = damping;
                 c.max_speed = max_speed;
                 c.weights = weights;
                 return c;
             }),
             py::arg("method") = mc::PriorityMethod::Recursive,
             py::arg("inverse") = mc::InverseMethod::Pseudoinverse, py::arg("damping") = 0.05,
             py::arg("max_speed") = std::numeric_limits<double>::infinity(), py::arg("weights") = Eigen::VectorXd())
        .def_readwrite("method", &mc::PriorityConfig::method)
        .def_readwrite("inverse", &mc::PriorityConfig::inverse)
        .def_readwrite("damping", &mc::PriorityConfig::damping)
        .def_readwrite("max_speed", &mc::PriorityConfig::max_speed)
        .def_readwrite("weights", &mc::PriorityConfig::weights);

    py::class_<mc::PriorityLevel>(m, "PriorityLevel")
        .def(py::init([](const Eigen::MatrixXd& J, const Eigen::VectorXd& v) { return mc::PriorityLevel{J, v}; }),
             py::arg("J"), py::arg("v"))
        .def_readwrite("J", &mc::PriorityLevel::J)
        .def_readwrite("v", &mc::PriorityLevel::v);

    m.def("task_priority_velocity", &mc::task_priority_velocity, py::arg("levels"), py::arg("n"),
          py::arg("config") = mc::PriorityConfig{}, "Command for levels ordered by priority (eqs. 7.5-7.7, 7.10).");
    m.def("hierarchy_projector", &mc::hierarchy_projector, py::arg("levels"), py::arg("n"), py::arg("count"),
          "Projector onto the common null space of the first `count` levels (eq. 7.6).");
    m.def("task_priority_step", &mc::task_priority_step, py::arg("model"), py::arg("q"), py::arg("tasks"),
          py::arg("config") = mc::PriorityConfig{}, "Update the tasks at q and return the command.");

    py::class_<mc::PriorityTrajectory>(m, "PriorityTrajectory")
        .def_readonly("t", &mc::PriorityTrajectory::t)
        .def_readonly("q", &mc::PriorityTrajectory::q)
        .def_readonly("u", &mc::PriorityTrajectory::u)
        .def_readonly("error_norm", &mc::PriorityTrajectory::error_norm)
        .def_readonly("active", &mc::PriorityTrajectory::active)
        .def_readonly("ee_position", &mc::PriorityTrajectory::ee_position);
    m.def("simulate_task_priority", &mc::simulate_task_priority, py::arg("model"), py::arg("q0"), py::arg("tasks"),
          py::arg("config"), py::arg("dt"), py::arg("steps"), "Simulate the task-priority closed loop.");
}
