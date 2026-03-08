#include "function.hpp"
#include <cmath>

double TargetFunction::operator()(double x) const {
    return x * x - 2 * x - 2 * std::cos(x);
}

std::string TargetFunction::name() const {
    return "f(x) = x^2 - 2x - 2cos(x)";
}