#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include "function.hpp"
#include "golden_section.hpp"
#include "uniform_search.hpp"


// Function to calculate theoretical number of iterations for golden section
int calculateGoldenSectionIterations(double initial_width, double epsilon) {
    const double golden_ratio = 0.6180339887498949;
    double ratio = initial_width / epsilon;
    return static_cast<int>(std::ceil(std::log(ratio) / std::log(1.0 / golden_ratio)));
}

// Function to calculate theoretical number of iterations for uniform search
int calculateUniformSearchIterations(double initial_width, double epsilon, int points_per_iter) {
    double ratio = initial_width / epsilon;
    return static_cast<int>(std::ceil(std::log(ratio) / std::log(points_per_iter - 1)));
}

int main() {
    // Create the target function
    TargetFunction func;

    // Define the interval
    double a = 0.5;
    double b = 1.0;

    // Define precision levels
    std::vector<double> epsilons = {0.1, 0.01, 0.001};

    std::cout << "Optimization of " << func.name() << " on [" << a << ", " << b << "]" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    for (double epsilon : epsilons) {
        std::cout << "\nPrecision: " << epsilon << std::endl;
        std::cout << std::string(40, '-') << std::endl;

        // Golden Section Method
        GoldenSectionOptimizer gs_optimizer;
        auto gs_result = gs_optimizer.minimize(func, a, b, epsilon);

        // Calculate theoretical iterations for golden section
        int gs_theoretical = calculateGoldenSectionIterations(b - a, epsilon);

        std::cout << "Golden Section Method:" << std::endl;
        std::cout << "  Minimum x: " << std::fixed << std::setprecision(6) << gs_result.min_x << std::endl;
        std::cout << "  Minimum value: " << std::fixed << std::setprecision(6) << gs_result.min_value << std::endl;
        std::cout << "  Function evaluations: " << gs_result.num_evaluations << std::endl;
        std::cout << "  Final interval width: " << gs_result.final_interval_width << std::endl;
        std::cout << "  Theoretical iterations needed: " << gs_theoretical << std::endl;
        std::cout << "  Actual iterations (approx): " << gs_result.num_evaluations - 2
                  << " (initial 2 evals + 1 per iter)" << std::endl;

        // Uniform Search Method
        UniformSearchOptimizer us_optimizer(5); // Using 5 points per iteration
        auto us_result = us_optimizer.minimize(func, a, b, epsilon);

        // Calculate theoretical iterations for uniform search
        int us_theoretical = calculateUniformSearchIterations(b - a, epsilon, 5);

        std::cout << "\nUniform Search Method (5 points per iteration):" << std::endl;
        std::cout << "  Minimum x: " << std::fixed << std::setprecision(6) << us_result.min_x << std::endl;
        std::cout << "  Minimum value: " << std::fixed << std::setprecision(6) << us_result.min_value << std::endl;
        std::cout << "  Function evaluations: " << us_result.num_evaluations << std::endl;
        std::cout << "  Final interval width: " << us_result.final_interval_width << std::endl;
        std::cout << "  Theoretical iterations needed: " << us_theoretical << std::endl;
        std::cout << "  Actual iterations (approx): " << (us_result.num_evaluations - 5) / 5 + 1
                  << " (5 evals per iter after first)" << std::endl;

        // Comparison
        std::cout << "\nComparison:" << std::endl;
        std::cout << "  Golden Section vs Uniform Search - Evaluations: " << gs_result.num_evaluations << " vs "
                  << us_result.num_evaluations << std::endl;

        if (gs_result.num_evaluations < us_result.num_evaluations) {
            std::cout << "  Golden Section is more efficient for this case." << std::endl;
        } else if (us_result.num_evaluations < gs_result.num_evaluations) {
            std::cout << "  Uniform Search is more efficient for this case." << std::endl;
        } else {
            std::cout << "  Both methods required the same number of evaluations." << std::endl;
        }
    }

    // Demonstrate unimodality by showing function values across the interval
    std::cout << "\n\nDemonstration of unimodality (function values across interval [0.5, 1.0]):" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(10) << "x" << std::setw(15) << "f(x)" << std::endl;
    std::cout << std::string(25, '-') << std::endl;

    for (int i = 0; i <= 20; ++i) {
        double x = 0.5 + i * (0.5 / 20.0);
        std::cout << std::fixed << std::setprecision(6) << std::setw(10) << x << std::setw(15) << func(x) << std::endl;
    }

    return 0;
}
