#ifndef UNIFORM_SEARCH_HPP
#define UNIFORM_SEARCH_HPP

#include "optimizer_base.hpp"

/**
 * @brief Optimizer using the uniform search method
 */
class UniformSearchOptimizer : public OptimizerBase {
private:
    int points_per_iteration;  ///< Number of points to evaluate per iteration

public:
    /**
     * @brief Constructor
     * @param points Number of points to use per iteration (default: 5)
     */
    explicit UniformSearchOptimizer(int points = 5);
    
    /**
     * @brief Minimize the given function using uniform search
     * @param func Function to minimize
     * @param a Left boundary of the interval
     * @param b Right boundary of the interval
     * @param epsilon Required precision
     * @return OptimizationResult containing the results
     */
    OptimizationResult minimize(
        const Function& func,
        double a, 
        double b, 
        double epsilon
    ) override;
};

#endif // UNIFORM_SEARCH_HPP