#include <gtest/gtest.h>
#include "uniform_search.hpp"
#include "function.hpp"
#include <cmath>

TEST(UniformSearchTest, BasicMinimization) {
    TargetFunction func;
    UniformSearchOptimizer optimizer(5);  // 5 points per iteration
    
    // Test minimization of target function in interval [0.5, 1.0]
    auto result = optimizer.minimize(func, 0.5, 1.0, 0.01);
    
    // The minimum should be somewhere in the interval
    EXPECT_GE(result.min_x, 0.5);
    EXPECT_LE(result.min_x, 1.0);
    EXPECT_GT(result.num_evaluations, 0);
    EXPECT_LE(result.final_interval_width, 0.01);
}

TEST(UniformSearchTest, DifferentPointsPerIteration) {
    TargetFunction func;
    
    // Test with different numbers of points per iteration
    UniformSearchOptimizer optimizer3(3);  // 3 points per iteration
    UniformSearchOptimizer optimizer5(5);  // 5 points per iteration
    UniformSearchOptimizer optimizer7(7);  // 7 points per iteration
    
    auto result3 = optimizer3.minimize(func, 0.5, 1.0, 0.1);
    auto result5 = optimizer5.minimize(func, 0.5, 1.0, 0.1);
    auto result7 = optimizer7.minimize(func, 0.5, 1.0, 0.1);
    
    // All should find a minimum in the interval
    EXPECT_GE(result3.min_x, 0.5);
    EXPECT_LE(result3.min_x, 1.0);
    EXPECT_GE(result5.min_x, 0.5);
    EXPECT_LE(result5.min_x, 1.0);
    EXPECT_GE(result7.min_x, 0.5);
    EXPECT_LE(result7.min_x, 1.0);
    
    // More points per iteration might lead to different evaluation counts
    EXPECT_GT(result3.num_evaluations, 0);
    EXPECT_GT(result5.num_evaluations, 0);
    EXPECT_GT(result7.num_evaluations, 0);
}

TEST(UniformSearchTest, EvaluationCount) {
    TargetFunction func;
    UniformSearchOptimizer optimizer(5);
    
    // Test that evaluation count is properly tracked
    auto result = optimizer.minimize(func, 0.5, 1.0, 0.1);
    
    // Should have more than 5 evaluations (at least 5 in first iteration + possible additional iterations)
    EXPECT_GT(result.num_evaluations, 4);
}

TEST(UniformSearchTest, PrecisionTest) {
    TargetFunction func;
    UniformSearchOptimizer optimizer(5);
    
    // Test with different precision levels
    auto result1 = optimizer.minimize(func, 0.5, 1.0, 0.1);
    auto result2 = optimizer.minimize(func, 0.5, 1.0, 0.01);
    auto result3 = optimizer.minimize(func, 0.5, 1.0, 0.001);
    
    // More precise results should have smaller intervals
    EXPECT_LE(result2.final_interval_width, result1.final_interval_width);
    EXPECT_LE(result3.final_interval_width, result2.final_interval_width);
    
    // More precise results may require more evaluations
    EXPECT_GE(result3.num_evaluations, result1.num_evaluations);
}

TEST(UniformSearchTest, ConstructorValidation) {
    // Test that constructor properly validates the number of points
    UniformSearchOptimizer optimizer_default;  // Should use 5 points
    UniformSearchOptimizer optimizer_min(2);   // Should be adjusted to 3 points (minimum)
    UniformSearchOptimizer optimizer_valid(10); // Should use 10 points
    
    // We can't directly access points_per_iteration member, but we can verify
    // that objects are created without errors
    EXPECT_NO_THROW({
        auto result1 = optimizer_default.minimize(TargetFunction(), 0.5, 1.0, 0.1);
    });
    
    EXPECT_NO_THROW({
        auto result2 = optimizer_min.minimize(TargetFunction(), 0.5, 1.0, 0.1);
    });
    
    EXPECT_NO_THROW({
        auto result3 = optimizer_valid.minimize(TargetFunction(), 0.5, 1.0, 0.1);
    });
}