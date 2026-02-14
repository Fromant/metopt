cmake_minimum_required(VERSION 3.31)
project(lp_test)

set(CMAKE_CXX_STANDARD 20)
include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
)

FetchContent_MakeAvailable(googletest)

file(GLOB_RECURSE TEST_SOURCES "*.cpp")
add_executable(lp_test ${TEST_SOURCES})

target_include_directories(lp_test PRIVATE ../src)

target_link_libraries(lp_test PRIVATE GTest::gtest_main lp)