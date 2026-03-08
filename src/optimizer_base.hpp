#ifndef OPTIMIZER_BASE_HPP
#define OPTIMIZER_BASE_HPP

#include "function.hpp"

/**
 * @brief Structure to hold optimization results
 */
struct OptimizationResult {
    double min_x;                ///< x-value at minimum
    double min_value;            ///< function value at minimum
    int num_evaluations;         ///< number of function evaluations
    double final_interval_width; ///< width of final interval
};

/**
 * @brief Abstract base class for optimization algorithms
 */
class OptimizerBase {
protected:
    int evaluation_count;  ///< Counter for function evaluations

public:
    OptimizerBase() : evaluation_count(0) {}
    virtual ~OptimizerBase() = default;
    
    /**
     * @brief Minimize the given function within the specified interval
     * @param func Function to minimize
     * @param a Left boundary of the interval
     * @param b Right boundary of the interval
     * @param epsilon Required precision
     * @return OptimizationResult containing the results
     */
    virtual OptimizationResult minimize(
        const Function& func,
        double a, 
        double b, 
        double epsilon
    ) = 0;
    
    /**
     * @brief Get the number of function evaluations performed
     * @return Number of function evaluations
     */
    int get_evaluation_count() const { return evaluation_count; }
    
    /**
     * @brief Reset the evaluation counter
     */
    void reset_evaluation_count() { evaluation_count = 0; }
    
protected:
    /**
     * @brief Increment the evaluation counter
     */
    void increment_evaluation_count() { evaluation_count++; }
};

#endif // OPTIMIZER_BASE_HPP