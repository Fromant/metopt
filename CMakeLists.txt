cmake_minimum_required(VERSION 3.22)

project(metopt)

set(CMAKE_CXX_STANDARD 20)
include(FetchContent)
FetchContent_Declare(
        Eigen3
        URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar
        DOWNLOAD_EXTRACT_TIMESTAMP true
)

# Disable tests and docs in eigen
set(BUILD_TESTING OFF CACHE INTERNAL "")
set(EIGEN_BUILD_TESTING OFF CACHE INTERNAL "")
set(EIGEN_BUILD_DOC OFF CACHE INTERNAL "")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(Eigen3)

add_executable(metopt main.cpp)

target_link_libraries(metopt PUBLIC Eigen3::Eigen)