#ifndef GOLDEN_SECTION_HPP
#define GOLDEN_SECTION_HPP

#include "optimizer_base.hpp"

/**
 * @brief Optimizer using the golden section search method
 */
class GoldenSectionOptimizer : public OptimizerBase {
private:
    static constexpr double GOLDEN_RATIO = 0.6180339887498949; // (sqrt(5)-1)/2
    static constexpr double INV_GOLDEN_RATIO = 1.0 - GOLDEN_RATIO; // 1 - (sqrt(5)-1)/2 = (3-sqrt(5))/2

public:
    /**
     * @brief Minimize the given function using golden section search
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

#endif // GOLDEN_SECTION_HPP