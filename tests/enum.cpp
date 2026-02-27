#include <gtest/gtest.h>

#include "linear/DualBuilder.hpp"
#include "linear/Solvers/EnumSolver.hpp"

const std::string files_folder = "../../tasks/";
constexpr auto EPS = 1e-9;


TEST(EnumSolverTest, file1) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task1.txt");
    const auto res = EnumSolver::solve(lp);
    EXPECT_TRUE(res.is_unbounded);
    EXPECT_FALSE(res.is_degenerate);
    EXPECT_TRUE(res.is_feasible);
    EXPECT_FALSE(res.is_optimal);
}

TEST(EnumSolverTest, file2) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task2.txt");
    const auto res = EnumSolver::solve(lp);
    EXPECT_FALSE(res.is_unbounded);
    EXPECT_TRUE(res.is_degenerate);
    EXPECT_TRUE(res.is_feasible);
    EXPECT_TRUE(res.is_optimal);
    EXPECT_NEAR(res.objective_value, 7, EPS);
    const std::vector<double> sol{0, 0, 0, -1, 2};
    for (int i = 0; i < 5; i++) {
        EXPECT_NEAR(res.x[i], sol[i], EPS);
    }
}

TEST(EnumSolverTest, file3) {
    const LinearProgram lp = LinearProgram::read_from_file(files_folder + "task3.txt");
    const auto res = EnumSolver::solve(lp);
    EXPECT_FALSE(res.is_unbounded);
    EXPECT_FALSE(res.is_degenerate);
    EXPECT_TRUE(res.is_feasible);
    EXPECT_TRUE(res.is_optimal);
    EXPECT_NEAR(res.objective_value, 1, EPS);
    const std::vector<double> sol{1, 0};
    for (int i = 0; i < 2; i++) {
        EXPECT_NEAR(res.x[i], sol[i], EPS);
    }
}

TEST(EnumSolverTest, file2_dual) {
    const LinearProgram lp = DualBuilder::build_dual(LinearProgram::read_from_file(files_folder + "task2.txt"));
    const auto res = EnumSolver::solve(lp);
    EXPECT_FALSE(res.is_unbounded);
    EXPECT_TRUE(res.is_degenerate);
    EXPECT_TRUE(res.is_feasible);
    EXPECT_TRUE(res.is_optimal);
    EXPECT_NEAR(res.objective_value, 7, EPS);
}

TEST(EnumSolverTest, file3_dual) {
    const LinearProgram lp = DualBuilder::build_dual(LinearProgram::read_from_file(files_folder + "task3.txt"));
    const auto res = EnumSolver::solve(lp);
    EXPECT_FALSE(res.is_unbounded);
    EXPECT_TRUE(res.is_feasible);
    EXPECT_TRUE(res.is_optimal);
    EXPECT_NEAR(res.objective_value, 1, EPS);
}
