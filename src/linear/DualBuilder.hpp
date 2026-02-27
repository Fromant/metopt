#pragma once
#include "LinearProgram.hpp"

/**
 * DualBuilder - строит двойственную задачу
 */
class DualBuilder {
public:
    static LinearProgram build_dual(LinearProgram);
};
