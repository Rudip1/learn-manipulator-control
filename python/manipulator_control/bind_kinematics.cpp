// Bindings for chapter 2: forward kinematics (DH and product of exponentials) and the example arms.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "manipulator_control/kinematics.hpp"
#include "manipulator_control/robots.hpp"

namespace py = pybind11;
namespace mc = manipulator_control;

void bind_kinematics(py::module_& m) {
    py::enum_<mc::JointType>(m, "JointType")
        .value("Revolute", mc::JointType::Revolute)
        .value("Prismatic", mc::JointType::Prismatic);

    py::class_<mc::DHLink>(m, "DHLink", "One link in standard DH form (section 2.2).")
        .def(py::init([](double a, double alpha, double d, double theta, mc::JointType type) {
                 return mc::DHLink{a, alpha, d, theta, type};
             }),
             py::arg("a") = 0.0, py::arg("alpha") = 0.0, py::arg("d") = 0.0, py::arg("theta") = 0.0,
             py::arg("type") = mc::JointType::Revolute)
        .def_readwrite("a", &mc::DHLink::a)
        .def_readwrite("alpha", &mc::DHLink::alpha)
        .def_readwrite("d", &mc::DHLink::d)
        .def_readwrite("theta", &mc::DHLink::theta)
        .def_readwrite("type", &mc::DHLink::type)
        .def("__repr__", [](const mc::DHLink& l) {
            std::ostringstream os;
            os << "DHLink(a=" << l.a << ", alpha=" << l.alpha << ", d=" << l.d << ", theta=" << l.theta
               << ", type=" << (l.type == mc::JointType::Revolute ? "Revolute" : "Prismatic") << ")";
            return os.str();
        });

    m.def("dh_transform", &mc::dh_transform, py::arg("a"), py::arg("alpha"), py::arg("d"), py::arg("theta"),
          "Link transform Rz(theta) Tz(d) Tx(a) Rx(alpha) (eq. 2.1).");

    py::class_<mc::SerialChain>(m, "SerialChain", "Serial chain described by DH parameters (section 2.2).")
        .def(py::init<std::vector<mc::DHLink>, const Eigen::Matrix4d&, const Eigen::Matrix4d&>(),
             py::arg("links"), py::arg("base") = Eigen::Matrix4d::Identity(),
             py::arg("tool") = Eigen::Matrix4d::Identity())
        .def_property_readonly("dof", &mc::SerialChain::dof)
        .def_property_readonly("links", &mc::SerialChain::links)
        .def_property("base", &mc::SerialChain::base, &mc::SerialChain::set_base)
        .def_property("tool", &mc::SerialChain::tool, &mc::SerialChain::set_tool)
        .def_property_readonly("joint_types", &mc::SerialChain::joint_types)
        .def("link_transform", &mc::SerialChain::link_transform, py::arg("i"), py::arg("qi"),
             "^{i-1}T_i(q_i), i = 1..n (eq. 2.1).")
        .def("frames", &mc::SerialChain::frames, py::arg("q"),
             "[T_0 = base, T_1, ..., T_n, T_e = T_n tool] in the base frame (eq. 2.2).")
        .def("end_effector", &mc::SerialChain::end_effector, py::arg("q"), "End-effector pose (eq. 2.2).");

    py::class_<mc::PoeChain>(m, "PoeChain", "Home pose M and space-frame screw axes (section 2.3).")
        .def(py::init([](const Eigen::Matrix4d& M, const std::vector<mc::Vector6d>& screws) {
                 return mc::PoeChain{M, screws};
             }),
             py::arg("M"), py::arg("screws"))
        .def_readwrite("M", &mc::PoeChain::M)
        .def_readwrite("screws", &mc::PoeChain::screws);

    m.def("poe_space", &mc::poe_space, py::arg("chain"), py::arg("q"),
          "T = exp([S1] q1) ... exp([Sn] qn) M (eq. 2.4).");
    m.def("poe_body", &mc::poe_body, py::arg("chain"), py::arg("q"), "T = M exp([B1] q1) ... (eq. 2.5).");
    m.def("body_screws", &mc::body_screws, py::arg("chain"), "B_i = Ad_{M^-1} S_i (eq. 2.5).");
    m.def("to_poe", &mc::to_poe, py::arg("chain"), "Product-of-exponentials form of a DH chain (eq. 2.7).");

    py::module_ robots = m.def_submodule("robots", "Example arms.");
    robots.def("planar", &mc::robots::planar, py::arg("lengths"), "Planar arm with revolute joints.");
    robots.def("puma560", &mc::robots::puma560, "PUMA 560, standard DH parameters.");
    robots.def("stanford", &mc::robots::stanford, "Stanford-type arm (RRP + spherical wrist).");
}
