cmake_minimum_required(VERSION 3.31)
project(metopt)

set(CMAKE_CXX_STANDARD 20)

add_executable(metopt
        src/main.cpp
        src/LinearProgram.cpp
        src/FormConverter.cpp
        src/DualBuilder.cpp)
