#include <gtest/gtest.h>
#include "golden_section.hpp"
#include "function.hpp"
#include <cmath>

class QuadraticFunction : public Function {
public:
    double operator()(double x) const override {
        return (x - 2.0) * (x - 2.0);  // Minimum at x = 2
    }
    
    std::string name() const override {
        return "QuadraticFunction";
    }
};

TEST(GoldenSectionTest, BasicMinimization) {
    QuadraticFunction func;
    GoldenSectionOptimizer optimizer;
    
    // Test minimization of quadratic function in interval [0, 5]
    // Minimum should be at x = 2
    auto result = optimizer.minimize(func, 0.0, 5.0, 0.01);
    
    EXPECT_NEAR(result.min_x, 2.0, 0.01);
    EXPECT_NEAR(result.min_value, 0.0, 0.01);
    EXPECT_GT(result.num_evaluations, 0);
    EXPECT_LE(result.final_interval_width, 0.01);
}

TEST(GoldenSectionTest, TargetFunctionMinimization) {
    TargetFunction func;
    GoldenSectionOptimizer optimizer;
    
    // Test minimization of target function f(x) = x^2 - 2x - 2cos(x) in interval [0.5, 1.0]
    auto result = optimizer.minimize(func, 0.5, 1.0, 0.01);
    
    // The minimum should be somewhere in the interval
    EXPECT_GE(result.min_x, 0.5);
    EXPECT_LE(result.min_x, 1.0);
    EXPECT_GT(result.num_evaluations, 0);
    EXPECT_LE(result.final_interval_width, 0.01);
}

TEST(GoldenSectionTest, EvaluationCount) {
    TargetFunction func;
    GoldenSectionOptimizer optimizer;
    
    // Test that evaluation count is properly tracked
    auto result = optimizer.minimize(func, 0.5, 1.0, 0.1);
    
    // Should have more than 2 evaluations (at least 2 initial + some iterations)
    EXPECT_GT(result.num_evaluations, 2);
}

TEST(GoldenSectionTest, PrecisionTest) {
    TargetFunction func;
    GoldenSectionOptimizer optimizer;
    
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