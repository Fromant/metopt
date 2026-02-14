#include <gtest/gtest.h>

#include "DualBuilder.hpp"
#include "SimplexSolver2.hpp"

const std::string files_folder = "../../tasks/";
constexpr auto EPS = 1e-9;

TEST(SimplexSolver2Test, file1) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task1.txt");
    const auto res = SimplexSolver2::solve(lp);
    EXPECT_TRUE(!res.is_feasible || res.is_unbounded);
}

TEST(SimplexSolver2Test, file2) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task2.txt");
    const auto res = SimplexSolver2::solve(lp);
    if (!res.is_feasible || res.is_unbounded) {
        FAIL();
    }
    EXPECT_NEAR(res.objective_value, 7, EPS);
    std::vector<double> sol{0, 0, 0, -1, 2};
    EXPECT_EQ(res.x.size(), sol.size());
    for (int i = 0; i < 5; i++) {
        EXPECT_NEAR(res.x[i], sol[i], EPS);
    }
}

TEST(SimplexSolver2Test, file3) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task3.txt");
    const auto res = SimplexSolver2::solve(lp);
    if (!res.is_feasible || res.is_unbounded) {
        FAIL();
    }
    EXPECT_NEAR(res.objective_value, 1, EPS);
    const std::vector<double> sol{1, 0};
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(res.x[i], sol[i], EPS);
    }
}

TEST(SimplexSolver2Test, file2_dual){
    const LinearProgram lp = DualBuilder::build_dual(LinearProgram::read_from_file(files_folder + "task2.txt"));
    const auto res = SimplexSolver2::solve(lp);
    if (!res.is_feasible || res.is_unbounded) {
        FAIL();
    }
    EXPECT_NEAR(res.objective_value, 7, EPS);
    std::vector<double> sol{0, 0, 0, -1, 2};
}

TEST(SimplexSolver2Test, file3_dual) {
    const LinearProgram lp = DualBuilder::build_dual(LinearProgram::read_from_file(files_folder + "task3.txt"));
    const auto res = SimplexSolver2::solve(lp);
    if (!res.is_feasible|| res.is_unbounded) {
        FAIL();
    }
    EXPECT_NEAR(res.objective_value, 1, EPS);
    const std::vector<double> sol{1, 0};
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(res.x[i], sol[i], EPS);
    }
}