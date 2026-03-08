#include <gtest/gtest.h>
#include "function.hpp"
#include <cmath>

TEST(TargetFunctionTest, BasicEvaluation) {
    TargetFunction func;
    
    // Test at x = 0
    EXPECT_DOUBLE_EQ(func(0), 0*0 - 2*0 - 2*std::cos(0));
    EXPECT_DOUBLE_EQ(func(0), -2);  // cos(0) = 1, so 0 - 0 - 2*1 = -2
    
    // Test at x = 1
    EXPECT_DOUBLE_EQ(func(1), 1*1 - 2*1 - 2*std::cos(1));
    EXPECT_NEAR(func(1), 1 - 2 - 2*std::cos(1), 1e-10);
    
    // Test at x = 2
    EXPECT_DOUBLE_EQ(func(2), 2*2 - 2*2 - 2*std::cos(2));
    EXPECT_NEAR(func(2), 4 - 4 - 2*std::cos(2), 1e-10);
    EXPECT_NEAR(func(2), -2*std::cos(2), 1e-10);
}

TEST(TargetFunctionTest, Name) {
    TargetFunction func;
    EXPECT_EQ(func.name(), "f(x) = x^2 - 2x - 2cos(x)");
}