#include "uniform_search.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

UniformSearchOptimizer::UniformSearchOptimizer(int points) 
    : points_per_iteration(points < 3 ? 3 : points) {}

OptimizationResult UniformSearchOptimizer::minimize(
    const Function& func,
    double a, 
    double b, 
    double epsilon
) {
    reset_evaluation_count();
    
    double current_a = a;
    double current_b = b;
    
    while (std::abs(current_b - current_a) > epsilon) {
        // Calculate step size
        double step = (current_b - current_a) / (points_per_iteration - 1);
        
        // Create points and evaluate function at each point
        std::vector<std::pair<double, double>> points_values;
        
        for (int i = 0; i < points_per_iteration; ++i) {
            double x = current_a + i * step;
            double fx = func(x);
            increment_evaluation_count();
            points_values.emplace_back(x, fx);
        }
        
        // Find the point with minimum function value
        auto min_it = std::min_element(points_values.begin(), points_values.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.second < b.second;
            });
        
        // Find the index of the minimum element
        int min_idx = std::distance(points_values.begin(), min_it);
        
        // Create a new interval around the minimum point
        // If minimum is at the boundary, expand appropriately
        double new_a, new_b;
        if (min_idx == 0) {
            // Minimum is at leftmost point
            new_a = points_values[0].first;
            new_b = points_values[1].first;
        } else if (min_idx == points_per_iteration - 1) {
            // Minimum is at rightmost point
            new_a = points_values[points_per_iteration - 2].first;
            new_b = points_values[points_per_iteration - 1].first;
        } else {
            // Minimum is in the middle, use adjacent points to form new interval
            new_a = points_values[min_idx - 1].first;
            new_b = points_values[min_idx + 1].first;
        }
        
        // Update the search interval
        current_a = new_a;
        current_b = new_b;
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