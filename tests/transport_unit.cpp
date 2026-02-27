#include <gtest/gtest.h>

#include "transport/TransportProblem.hpp"

constexpr auto EPS = 1e-9;

class TransportUnitTest : public ::testing::Test {
protected:
    TransportProblem createSimpleProblem() {
        TransportProblem tp;
        tp.m = 2;
        tp.n = 2;
        tp.supply = {10, 20};
        tp.demand = {15, 15};
        tp.cost = {{1, 2}, {3, 4}};
        return tp;
    }
    
    TransportProblem createUnbalancedProblem() {
        TransportProblem tp;
        tp.m = 2;
        tp.n = 2;
        tp.supply = {20, 20};
        tp.demand = {15, 15};
        tp.cost = {{1, 2}, {3, 4}};
        return tp;
    }
};

TEST_F(TransportUnitTest, validate_correct) {
    TransportProblem tp = createSimpleProblem();
    EXPECT_TRUE(tp.validate());
}

TEST_F(TransportUnitTest, validate_zero_dimensions) {
    TransportProblem tp;
    tp.m = 0;
    tp.n = 2;
    tp.supply = {};
    tp.demand = {1, 2};
    tp.cost = {};
    EXPECT_FALSE(tp.validate());
}

TEST_F(TransportUnitTest, validate_negative_supply) {
    TransportProblem tp;
    tp.m = 1;
    tp.n = 1;
    tp.supply = {-5};
    tp.demand = {5};
    tp.cost = {{1}};
    EXPECT_FALSE(tp.validate());
}

TEST_F(TransportUnitTest, validate_negative_demand) {
    TransportProblem tp;
    tp.m = 1;
    tp.n = 1;
    tp.supply = {5};
    tp.demand = {-5};
    tp.cost = {{1}};
    EXPECT_FALSE(tp.validate());
}

TEST_F(TransportUnitTest, isBalanced_true) {
    TransportProblem tp = createSimpleProblem();
    EXPECT_TRUE(tp.isBalanced());
}

TEST_F(TransportUnitTest, isBalanced_false) {
    TransportProblem tp = createUnbalancedProblem();
    EXPECT_FALSE(tp.isBalanced());
}

TEST_F(TransportUnitTest, balance_adds_dummy_consumer) {
    TransportProblem tp = createUnbalancedProblem();
    TransportProblem balanced = tp.balance();
    
    EXPECT_EQ(balanced.m, 2u);
    EXPECT_EQ(balanced.n, 3u);
    EXPECT_EQ(balanced.supply.size(), 2u);
    EXPECT_EQ(balanced.demand.size(), 3u);
    EXPECT_EQ(balanced.cost.size(), 2u);
    EXPECT_EQ(balanced.cost[0].size(), 3u);
    EXPECT_TRUE(balanced.isBalanced());
}

TEST_F(TransportUnitTest, balance_adds_dummy_supplier) {
    TransportProblem tp;
    tp.m = 2;
    tp.n = 2;
    tp.supply = {15, 15};
    tp.demand = {20, 20};
    tp.cost = {{1, 2}, {3, 4}};
    
    TransportProblem balanced = tp.balance();
    
    EXPECT_EQ(balanced.m, 3u);
    EXPECT_EQ(balanced.n, 2u);
    EXPECT_TRUE(balanced.isBalanced());
}

TEST_F(TransportUnitTest, balance_already_balanced) {
    TransportProblem tp = createSimpleProblem();
    TransportProblem balanced = tp.balance();
    
    EXPECT_EQ(balanced.m, tp.m);
    EXPECT_EQ(balanced.n, tp.n);
    EXPECT_TRUE(balanced.isBalanced());
}

TEST_F(TransportUnitTest, calculateCost_correct) {
    TransportProblem tp = createSimpleProblem();
    std::vector<std::vector<double>> plan = {{10, 0}, {5, 15}};
    
    double cost = tp.calculateCost(plan);
    double expected = 10*1 + 0*2 + 5*3 + 15*4;
    EXPECT_NEAR(cost, expected, EPS);
}

TEST_F(TransportUnitTest, calculateCost_zero) {
    TransportProblem tp = createSimpleProblem();
    std::vector<std::vector<double>> plan = {{0, 0}, {0, 0}};
    
    double cost = tp.calculateCost(plan);
    EXPECT_NEAR(cost, 0.0, EPS);
}

TEST_F(TransportUnitTest, restorePlanFromVector_correct) {
    std::vector<double> x_vector = {1, 2, 3, 4};
    size_t m = 2, n = 2;
    
    auto plan = TransportProblem::restorePlanFromVector(x_vector, m, n);
    
    EXPECT_EQ(plan.size(), m);
    EXPECT_EQ(plan[0].size(), n);
    EXPECT_NEAR(plan[0][0], 1, EPS);
    EXPECT_NEAR(plan[0][1], 2, EPS);
    EXPECT_NEAR(plan[1][0], 3, EPS);
    EXPECT_NEAR(plan[1][1], 4, EPS);
}

TEST_F(TransportUnitTest, restorePlanFromVector_wrong_size) {
    std::vector<double> x_vector = {1, 2, 3};
    size_t m = 2, n = 2;
    
    EXPECT_THROW(TransportProblem::restorePlanFromVector(x_vector, m, n), std::runtime_error);
}

TEST_F(TransportUnitTest, toLinearProgram_balanced) {
    TransportProblem tp = createSimpleProblem();
    LinearProgram lp = tp.toLinearProgram();
    
    EXPECT_EQ(lp.num_variables(), 4u);
    EXPECT_EQ(lp.num_constraints(), 3u);
}

TEST_F(TransportUnitTest, toLinearProgram_unbalanced_throws) {
    TransportProblem tp = createUnbalancedProblem();
    
    EXPECT_THROW(tp.toLinearProgram(), std::runtime_error);
}

TEST_F(TransportUnitTest, toLinearProgram_balanced_first) {
    TransportProblem tp = createUnbalancedProblem();
    TransportProblem balanced = tp.balance();
    LinearProgram lp = balanced.toLinearProgram();
    
    EXPECT_EQ(lp.num_variables(), 6u);
    EXPECT_EQ(lp.num_constraints(), 4u);
}

TEST_F(TransportUnitTest, toLinearProgram_objective_coefficients) {
    TransportProblem tp;
    tp.m = 2;
    tp.n = 2;
    tp.supply = {10, 10};
    tp.demand = {10, 10};
    tp.cost = {{1, 2}, {3, 4}};
    
    LinearProgram lp = tp.toLinearProgram();
    
    std::vector<double> expected = {1, 2, 3, 4};
    for (size_t i = 0; i < expected.size(); i++) {
        EXPECT_NEAR(lp.objective()[i], expected[i], EPS);
    }
}

TEST_F(TransportUnitTest, supply_demand_vectors) {
    TransportProblem tp;
    tp.m = 3;
    tp.n = 2;
    tp.supply = {10, 20, 30};
    tp.demand = {40, 20};
    tp.cost = {{1, 2}, {3, 4}, {5, 6}};
    
    EXPECT_EQ(tp.supply.size(), 3u);
    EXPECT_EQ(tp.demand.size(), 2u);
    EXPECT_EQ(tp.cost.size(), 3u);
    EXPECT_EQ(tp.cost[0].size(), 2u);
}
