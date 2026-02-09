#include "DualBuilder.hpp"
#include <cmath>
#include <string>
#include <vector>

#include "FormConverter.hpp"

LinearProgram DualBuilder::build_dual(LinearProgram problem) {
    if (problem.getForm() == LinearProgram::UNDEFINED) {
        problem = FormConverter::to_general_form(problem);
    }

    // Общая форма всегда на минимизацию => обратная - на максимизацию
    bool dual_minimize = false;

    size_t vars = problem.num_constraints();
    size_t constraints = problem.num_variables();


    // c' = b
    const std::vector<double>& dual_objective = problem.rhs();

    // A' = A^T
    std::vector<std::vector<double>> dual_constraints(constraints, std::vector<double>(vars, 0.0));
    for (size_t i = 0; i < vars; ++i) {
        for (size_t j = 0; j < constraints; ++j) {
            dual_constraints[j][i] = problem.constraints()[i][j];
        }
    }

    // b' = c;
    const std::vector<double>& dual_rhs = problem.objective();


    const auto M1 = problem.getM1();
    const auto M2 = problem.getM2();
    const auto N1 = problem.getN1();
    const auto N2 = problem.getN2();

    std::vector<std::string> dual_relations(problem.num_variables());
    for (const auto& i : N1) {
        dual_relations[i] = "<=";
    }
    for (const auto& i : N2) {
        dual_relations[i] = "=";
    }

    std::vector<std::string> dual_var_constraints(problem.num_constraints());
    for (const auto& i : M1) {
        dual_var_constraints[i] = ">=0";
    }
    for (const auto& i : M2) {
        dual_var_constraints[i] = "free";
    }


    return {dual_minimize, dual_objective, dual_constraints, dual_relations, dual_rhs, dual_var_constraints};
}
