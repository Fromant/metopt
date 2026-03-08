#include "golden_section.hpp"
#include <cmath>

OptimizationResult GoldenSectionOptimizer::minimize(
    const Function& func,
    double a, 
    double b, 
    double epsilon
) {
    reset_evaluation_count();
    
    double x1, x2;
    double f1, f2;
    
    // Initialize the interval
    double current_a = a;
    double current_b = b;
    
    // Calculate initial internal points
    double d = GOLDEN_RATIO * (current_b - current_a);
    x1 = current_a + (1.0 - GOLDEN_RATIO) * (current_b - current_a);
    x2 = current_a + GOLDEN_RATIO * (current_b - current_a);
    
    // Evaluate function at initial points
    f1 = func(x1);
    increment_evaluation_count();
    f2 = func(x2);
    increment_evaluation_count();
    
    // Iterate until the interval is smaller than epsilon
    while (std::abs(current_b - current_a) > epsilon) {
        if (f1 <= f2) {
            // Minimum is in [a, x2], so update b = x2 and x2 = x1
            current_b = x2;
            x2 = x1;
            f2 = f1;
            
            // Calculate new x1
            x1 = current_a + (1.0 - GOLDEN_RATIO) * (current_b - current_a);
            f1 = func(x1);
            increment_evaluation_count();
        } else {
            // Minimum is in [x1, b], so update a = x1 and x1 = x2
            current_a = x1;
            x1 = x2;
            f1 = f2;
            
            // Calculate new x2
            x2 = current_a + GOLDEN_RATIO * (current_b - current_a);
            f2 = func(x2);
            increment_evaluation_count();
        }
    }
    
    // Return the midpoint of the final interval as the minimum
    double min_x = (current_a + current_b) / 2.0;
    double min_value = func(min_x);
    increment_evaluation_count(); // Count the final evaluation
    
    OptimizationResult result;
    result.min_x = min_x;
    result.min_value = min_value;
    result.num_evaluations = get_evaluation_count();
    result.final_interval_width = std::abs(current_b - current_a);
    
    return result;
}