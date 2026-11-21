foreach(example
    01_transforms
    02_forward_kinematics
    03_jacobian
    04_resolved_rate
    05_singularities
    06_redundancy
    07_task_priority
    08_inequality_tasks
)
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PRIVATE manipulator_control)
endforeach()
