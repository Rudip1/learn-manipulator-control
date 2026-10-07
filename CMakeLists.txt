cmake_minimum_required(VERSION 3.20)
project(manipulator_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MC_BUILD_TESTS "Build the Catch2 test suite" ${PROJECT_IS_TOP_LEVEL})
option(MC_BUILD_EXAMPLES "Build the command-line examples" ${PROJECT_IS_TOP_LEVEL})
option(MC_BUILD_PYTHON "Build the pybind11 module" OFF)

include(FetchContent)

# Eigen 3.4: use the system copy when there is one (apt install libeigen3-dev), otherwise fetch the
# headers. Only the headers are needed, so the fetched tree is not configured as a CMake project.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen 3.4 not found on the system; fetching it")
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
        SOURCE_SUBDIR headers-only)
    FetchContent_MakeAvailable(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

if(MC_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(cpp)

if(MC_BUILD_PYTHON)
    add_subdirectory(python)
endif()
