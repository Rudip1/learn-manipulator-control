find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core MODULE
    manipulator_control/bindings.cpp
    manipulator_control/bind_transforms.cpp
    manipulator_control/bind_kinematics.cpp
    manipulator_control/bind_jacobian.cpp
    manipulator_control/bind_resolved_rate.cpp
    manipulator_control/bind_singularities.cpp
)
target_link_libraries(_core PRIVATE manipulator_control)
install(TARGETS _core LIBRARY DESTINATION manipulator_control)
