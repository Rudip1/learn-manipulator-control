add_library(manipulator_control
    src/transforms.cpp
    src/kinematics.cpp
    src/robots.cpp
    src/jacobian.cpp
    src/resolved_rate.cpp
    src/singularities.cpp
)
add_library(manipulator_control::manipulator_control ALIAS manipulator_control)
target_include_directories(manipulator_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(manipulator_control PUBLIC Eigen3::Eigen)
if(MSVC)
    target_compile_options(manipulator_control PRIVATE /W4)
else()
    target_compile_options(manipulator_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(MC_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(MC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
