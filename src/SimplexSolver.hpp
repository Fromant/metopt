#pragma once

#include "LinearProgram.hpp"
#include <vector>

class SimplexSolver {
public:
    struct Solution {
        bool feasible = false;
        bool bounded = true;
        std::vector<double> x;
        double objective_value = 0.0;
    };

    static Solution solve(const LinearProgram& lp);
};
