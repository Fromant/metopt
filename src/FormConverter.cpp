#include "FormConverter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

LinearProgram FormConverter::to_general_form(const LinearProgram& lp) {
    LinearProgram general = lp;
    for (size_t i = 0; i < lp.num_variables(); ++i) {
        if (lp.var_constraints()[i] == "<=0") {
            // b'[i] = -b[i]
            general.objective()[i] = -lp.objective()[i];
            // A'[i][j] = -A[i][j]
            for (size_t j = 0; j < lp.num_constraints(); ++j) {
                general.constraints()[i][j] = -lp.constraints()[i][j];
            }

            general.var_constraints()[i] = ">=0";
        }
    }

    // max -> min
    if (!lp.is_minimization()) {
        for (double& coeff : general.objective())
            coeff = -coeff;
        general.is_minimization() = true;
    }

    std::vector<std::vector<double>> new_constraints;
    std::vector<std::string> new_relations;
    std::vector<double> new_rhs;

    for (size_t i = 0; i < lp.num_constraints(); ++i) {
        if (lp.relations()[i] == "<=") {
            // a[i]x <= b[i] -> a[i]x >= b[i]
            std::vector<double> neg_row(lp.num_variables());
            for (size_t j = 0; j < lp.num_variables(); ++j) {
                neg_row[j] = -lp.constraints()[i][j];
            }
            new_constraints.emplace_back(neg_row);
            new_relations.emplace_back(">=");
            new_rhs.emplace_back(-lp.rhs()[i]);
        } else if (lp.relations()[i] == ">=" || lp.relations()[i] == "=") {
            new_constraints.emplace_back(lp.constraints()[i]);
            new_relations.emplace_back(lp.relations()[i]);
            new_rhs.emplace_back(lp.rhs()[i]);
        } else {
            throw std::runtime_error("Invalid constraint type: " + lp.relations()[i]);
        }
    }

    general.constraints() = std::move(new_constraints);
    general.relations() = std::move(new_relations);
    general.rhs() = std::move(new_rhs);
    general.prettierCoeffs();

    // Проверка корректности ограничений на переменные
    for (const auto& vc : general.var_constraints()) {
        if (vc != ">=0" && vc != "free") {
            throw std::runtime_error("Internal error: invalid variable constraint in general form: " + vc);
        }
    }

    return general;
}

LinearProgram FormConverter::to_symmetric_form(const LinearProgram& lp) {
    const LinearProgram general = to_general_form(lp);

    const auto M1 = general.getM1();
    const auto M2 = general.getM2();
    const auto N1 = general.getN1();
    const auto N2 = general.getN2();

    size_t original_vars = lp.num_variables();
    size_t free_vars = N2.size();
    size_t total_vars = original_vars + free_vars;

    size_t original_constraints = general.num_constraints();
    size_t total_constraints = original_constraints + M2.size();

    std::vector<std::string> new_var_constraints(total_vars, ">=0");

    // c = (c[N1], c[N2], -c[N2])
    std::vector<double> new_objective;
    new_objective.reserve(total_vars);
    for (const double& i : general.objective()) {
        new_objective.emplace_back(i);
    }
    for (const auto& i : N2) {
        new_objective.emplace_back(-general.objective()[i]);
    }

    // b = (b[M1], b[M2], -b[M2])
    std::vector<double> new_rhs;
    new_rhs.reserve(total_constraints);
    for (const double& i : general.rhs()) {
        new_rhs.emplace_back(i);
    }
    for (const auto& i : M2) {
        new_rhs.emplace_back(-general.rhs()[i]);
    }

    //     ( A[M1][N1],  A[M1][N2], -A[M1,N2])
    // A = ( A[M2][N1],  A[M2][N2], -A[M2,N2])
    //     (-A[M2][N1], -A[M2][N2],  A[M2,N2])
    auto new_constraints = general.constraints();
    for (size_t y = 0; y < general.constraints().size(); y++) {
        for (const auto& i : N2) {
            new_constraints[y].emplace_back(-general.constraints()[y][i]);
        }
    }
    for (size_t y = 0; y < M2.size(); y++) {
        new_constraints.emplace_back();
        new_constraints[y + original_constraints].reserve(total_vars);
        for (size_t x = 0; x < original_vars; x++) {
            new_constraints[y + original_constraints].emplace_back(-general.constraints()[M2[y]][x]);
        }
        for (const auto& i : N2) {
            new_constraints[y + original_constraints].emplace_back(general.constraints()[M2[y]][i]);
        }
    }

    const std::vector<std::string> new_relations(total_constraints, ">=");

    return {true, new_objective, new_constraints, new_relations, new_rhs, new_var_constraints};
}

LinearProgram FormConverter::to_canonical_form(const LinearProgram& lp) {
    LinearProgram general = to_general_form(lp);

    const auto M1 = general.getM1();
    const auto M2 = general.getM2();
    const auto N1 = general.getN1();
    const auto N2 = general.getN2();

    size_t n = general.num_variables();
    size_t m = general.num_constraints();

    size_t n_new = n + N2.size() + M1.size();
    size_t m_new = m;


    // c = (c[N1], c[N2], -c[N2], 0[M1])
    std::vector<double> new_objective;
    new_objective.reserve(n_new);

    for (size_t i = 0; i < n; i++) {
        new_objective.emplace_back(general.objective()[i]);
    }
    for (const auto& i : N2) {
        new_objective.emplace_back(-general.objective()[i]);
    }
    for (int i = 0; i < M1.size(); i++) {
        new_objective.emplace_back(0);
    }

    // A = (A[M1,N1], A[M1,N2], -A[M1,N2], -E[M1,M1]
    //     (A[M2,N1], A[M2,N2], -A[M2,N2],  0[M2,M1]

    auto new_constraints = general.constraints();
    for (size_t j = 0; j < M1.size(); j++) {
        const size_t y = M1[j];
        for (const auto& i : N2) {
            new_constraints[y].emplace_back(-general.constraints()[y][i]);
        }
        for (size_t i = 0; i < M1.size(); i++) {
            if (i == j) {
                new_constraints[y].emplace_back(-1);
            } else {
                new_constraints[y].emplace_back(0);
            }
        }
    }
    for (const auto& y : M2) {
        for (const auto& i : N2) {
            new_constraints[y].emplace_back(-general.constraints()[y][i]);
        }
        for (size_t i = 0; i < M1.size(); i++) {
            new_constraints[y].emplace_back(0);
        }
    }

    std::vector<double> new_rhs = general.rhs();

    std::vector<std::string> new_var_constraints(n_new, ">=0");

    LinearProgram result;
    result.is_minimization() = true;
    result.objective() = std::move(new_objective);
    result.constraints() = std::move(new_constraints);
    result.relations() = std::vector<std::string>(m_new, "=");
    result.rhs() = std::move(new_rhs);
    result.var_constraints() = std::move(new_var_constraints);

    result.prettierCoeffs();
    return result;
}
