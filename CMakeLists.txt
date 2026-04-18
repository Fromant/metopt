cmake_minimum_required(VERSION 3.14)
project(bilp_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Core solver library
add_library(solver_lib
    src/solver.cpp
)
target_include_directories(solver_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Main executable
add_executable(bilp_solver src/main.cpp)
target_link_libraries(bilp_solver PRIVATE solver_lib)

# Test executable
add_executable(bilp_tests tests/tests.cpp)
target_link_libraries(bilp_tests PRIVATE solver_lib)

# Enable warnings
target_compile_options(solver_lib PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)
target_compile_options(bilp_solver PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
target_compile_options(bilp_tests PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
