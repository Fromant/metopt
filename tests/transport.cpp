#include <gtest/gtest.h>

#include "linear/solvers/SimplexSolver.hpp"
#include "transport/NorthwestCorner.hpp"
#include "transport/TransportProblem.hpp"
#include "transport/MODISolver.hpp"

const std::string files_folder = "../../tasks/transport/";
constexpr auto EPS = 1e-4;

class TransportIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(TransportIntegrationTest, transport1_northwest_corner) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport1.txt");
    EXPECT_TRUE(tp.validate());
    EXPECT_TRUE(tp.isBalanced());
    
    auto result = NorthwestCorner::solve(tp, false);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.plan.size(), 2u);
    EXPECT_EQ(result.plan[0].size(), 2u);
    
    double cost = tp.calculateCost(result.plan);
    EXPECT_NEAR(cost, 60.0, EPS);
}

TEST_F(TransportIntegrationTest, transport1_modi_optimal) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport1.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    EXPECT_TRUE(modiResult.success);
    EXPECT_TRUE(modiResult.isOptimal);
    
    double cost = tp.calculateCost(modiResult.plan);
    EXPECT_NEAR(cost, 60.0, EPS);
}

TEST_F(TransportIntegrationTest, transport1_simplex_agrees) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport1.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    LinearProgram lp = tp.toLinearProgram();
    auto simplexResult = SimplexSolver::solve(lp, false);
    
    ASSERT_TRUE(simplexResult.is_feasible);
    ASSERT_TRUE(simplexResult.is_optimal);
    
    auto modiPlan = TransportProblem::restorePlanFromVector(simplexResult.x, tp.m, tp.n);
    double simplexCost = tp.calculateCost(modiPlan);
    double modiCost = tp.calculateCost(modiResult.plan);
    
    EXPECT_NEAR(simplexCost, modiCost, EPS);
}

TEST_F(TransportIntegrationTest, transport2_northwest_corner) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport2.txt");
    EXPECT_TRUE(tp.validate());
    EXPECT_TRUE(tp.isBalanced());
    
    auto result = NorthwestCorner::solve(tp, false);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.plan.size(), 3u);
    EXPECT_EQ(result.plan[0].size(), 3u);
    
    double cost = tp.calculateCost(result.plan);
    EXPECT_GT(cost, 0.0);
}

TEST_F(TransportIntegrationTest, transport2_modi_optimal) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport2.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    EXPECT_TRUE(modiResult.success);
    EXPECT_TRUE(modiResult.isOptimal);
}

TEST_F(TransportIntegrationTest, transport2_simplex_agrees) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport2.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    LinearProgram lp = tp.toLinearProgram();
    auto simplexResult = SimplexSolver::solve(lp, false);
    
    ASSERT_TRUE(simplexResult.is_feasible);
    ASSERT_TRUE(simplexResult.is_optimal);
    
    auto modiPlan = TransportProblem::restorePlanFromVector(simplexResult.x, tp.m, tp.n);
    double simplexCost = tp.calculateCost(modiPlan);
    double modiCost = tp.calculateCost(modiResult.plan);
    
    EXPECT_NEAR(simplexCost, modiCost, EPS);
}

TEST_F(TransportIntegrationTest, transport3_northwest_corner) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport3.txt");
    EXPECT_TRUE(tp.validate());
    EXPECT_TRUE(tp.isBalanced());
    
    auto result = NorthwestCorner::solve(tp, false);
    EXPECT_TRUE(result.success);
    
    double cost = tp.calculateCost(result.plan);
    EXPECT_GT(cost, 0.0);
}

TEST_F(TransportIntegrationTest, transport3_modi_optimal) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport3.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    EXPECT_TRUE(modiResult.success);
    EXPECT_TRUE(modiResult.isOptimal);
}

TEST_F(TransportIntegrationTest, transport3_simplex_agrees) {
    TransportProblem tp = TransportProblem::fromFile(files_folder + "transport3.txt");
    auto nwResult = NorthwestCorner::solve(tp, false);
    auto modiResult = MODISolver::solve(tp, nwResult.plan, nwResult.basis, false);
    
    LinearProgram lp = tp.toLinearProgram();
    auto simplexResult = SimplexSolver::solve(lp, false);
    
    ASSERT_TRUE(simplexResult.is_feasible);
    ASSERT_TRUE(simplexResult.is_optimal);
    
    auto modiPlan = TransportProblem::restorePlanFromVector(simplexResult.x, tp.m, tp.n);
    double simplexCost = tp.calculateCost(modiPlan);
    double modiCost = tp.calculateCost(modiResult.plan);
    
    EXPECT_NEAR(simplexCost, modiCost, EPS);
}
