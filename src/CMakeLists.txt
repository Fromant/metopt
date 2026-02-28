cmake_minimum_required(VERSION 3.31)
project(lp)

set(CMAKE_CXX_STANDARD 20)
include(FetchContent)

FetchContent_Declare(
        Eigen3
        URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar
)

# Disable tests and docs in eigen
set(BUILD_TESTING OFF CACHE INTERNAL "")
set(EIGEN_BUILD_TESTING OFF CACHE INTERNAL "")
set(EIGEN_BUILD_DOC OFF CACHE INTERNAL "")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(Eigen3)

add_library(lp
        linear/LinearProgram.cpp
        linear/FormConverter.cpp
        linear/DualBuilder.cpp
        linear/solvers/SimplexSolver.cpp
        linear/solvers/EnumSolver.cpp
        transport/TransportProblem.cpp
        transport/TransportProblem.hpp
        transport/TransportSolver.cpp
        transport/TransportSolver.hpp
)

target_include_directories(lp PUBLIC ./)

target_link_libraries(lp PUBLIC Eigen3::Eigen)

