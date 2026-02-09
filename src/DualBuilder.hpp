#pragma once
#ifndef DUAL_BUILDER_H
#define DUAL_BUILDER_H

#include "LinearProgram.hpp"

/**
 * DualBuilder - строит двойственную задачу
 */
class DualBuilder {
public:
    static LinearProgram build_dual(LinearProgram);
};

#endif // DUAL_BUILDER_H