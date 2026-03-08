#ifndef FUNCTION_HPP
#define FUNCTION_HPP

#include <string>

/**
 * @brief Abstract base class for mathematical functions
 */
class Function {
public:
    virtual ~Function() = default;
    /**
     * @brief Evaluate the function at point x
     * @param x Point at which to evaluate the function
     * @return Function value at x
     */
    virtual double operator()(double x) const = 0;
    
    /**
     * @brief Get the name/description of the function
     * @return Function name
     */
    virtual std::string name() const = 0;
};

/**
 * @brief Implementation of the target function: f(x) = x^2 - 2x - 2cos(x)
 */
class TargetFunction : public Function {
public:
    double operator()(double x) const override;
    std::string name() const override;
};

#endif // FUNCTION_HPP