#include "DualBuilder.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

LinearProgram DualBuilder::build_dual(const LinearProgram& general) {
    if (general.getForm() == LinearProgram::UNDEFINED) {
        throw std::runtime_error("Dual construction error: can construct only dual to general form");
    }

    // Общая форма всегда на минимизацию => обратная - на максимизацию
    bool dual_minimize = false;

    size_t vars = general.num_constraints();
    size_t constraints = general.num_variables();


    // c' = b
    const std::vector<double>& dual_objective = general.rhs();

    // A' = A^T
    std::vector<std::vector<double>> dual_constraints(constraints, std::vector<double>(vars, 0.0));
    for (size_t i = 0; i < vars; ++i) {
        for (size_t j = 0; j < constraints; ++j) {
            dual_constraints[j][i] = general.constraints()[i][j];
        }
    }

    // b' = c;
    const std::vector<double>& dual_rhs = general.objective();


    const auto M1 = general.getM1();
    const auto M2 = general.getM2();
    const auto N1 = general.getN1();
    const auto N2 = general.getN2();

    std::vector<std::string> dual_relations(general.num_variables());
    for (const auto& i : N1) {
        dual_relations[i] = "<=";
    }
    for (const auto& i : N2) {
        dual_relations[i] = "=";
    }

    std::vector<std::string> dual_var_constraints(general.num_constraints());
    for (const auto& i : M1) {
        dual_var_constraints[i] = ">=0";
    }
    for (const auto& i : M2) {
        dual_var_constraints[i] = "free";
    }


    return {dual_minimize, dual_objective, dual_constraints, dual_relations, dual_rhs, dual_var_constraints};
}
