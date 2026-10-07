# Catch2 v3: the system package when present, otherwise FetchContent.
find_package(Catch2 3 QUIET)
if(NOT TARGET Catch2::Catch2WithMain)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.7.1
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()
include(Catch)

add_executable(manipulator_control_tests
    test_transforms.cpp
    test_kinematics.cpp
    test_jacobian.cpp
    test_resolved_rate.cpp
    test_singularities.cpp
)
target_link_libraries(manipulator_control_tests PRIVATE manipulator_control Catch2::Catch2WithMain)
catch_discover_tests(manipulator_control_tests)
