cmake_minimum_required(VERSION 3.31)
project(metopt)

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

add_executable(metopt
        src/main.cpp
        src/LinearProgram.cpp
        src/FormConverter.cpp
        src/DualBuilder.cpp
        src/SimplexSolver.cpp
        src/EnumSolver.cpp
)

target_link_libraries(metopt PRIVATE Eigen3::Eigen)

