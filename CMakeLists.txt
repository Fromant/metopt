cmake_minimum_required(VERSION 3.10)
project(metopt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler-specific options
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

# Find required packages
find_package(Threads REQUIRED)

# Create the main executable
add_executable(metopt
    src/main.cpp
    src/function.cpp
    src/golden_section.cpp
    src/uniform_search.cpp
)

# Link libraries
target_link_libraries(metopt ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(metopt PRIVATE src)

# Create tests subdirectory
enable_testing()

# GoogleTest setup
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Create test executable
add_executable(tests
    tests/test_main.cpp
    tests/test_function.cpp
    tests/test_golden_section.cpp
    tests/test_uniform_search.cpp
    src/function.cpp
    src/golden_section.cpp
    src/uniform_search.cpp
)

target_link_libraries(tests gtest_main)
target_include_directories(tests PRIVATE src include)

# Add test executable to CTest
add_test(NAME UnitTests COMMAND tests)
