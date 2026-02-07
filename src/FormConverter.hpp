#pragma once

#include "LinearProgram.hpp"

class FormConverter {
public:
    static LinearProgram to_general_form(const LinearProgram& lp);

    static LinearProgram to_symmetric_form(const LinearProgram& lp);

    static LinearProgram to_canonical_form(const LinearProgram& lp);
};
