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

    return general;
}

LinearProgram FormConverter::to_symmetric_form(const LinearProgram& lp) {
    LinearProgram general = to_general_form(lp);

    const auto M1 = general.getM1();
    const auto M2 = general.getM2();
    const auto N1 = general.getN1();
    const auto N2 = general.getN2();

    size_t original_vars = lp.num_variables();
    size_t new_vars = N2.size();
    size_t total_vars = original_vars + new_vars;

    size_t original_constraints = general.num_constraints();
    size_t new_constraints = M2.size();
    size_t total_constraints = original_constraints + new_constraints;

    // c = (c[N1], c[N2], -c[N2])
    for (const auto& i : N2) {
        general.objective().emplace_back(-general.objective()[i]);
    }

    // b = (b[M1], b[M2], -b[M2])
    for (const auto& i : M2) {
        general.rhs().emplace_back(-general.rhs()[i]);
    }


    auto& constraints = general.constraints();

    //     ( A[M1][N1],  A[M1][N2], -A[M1,N2])
    // A = ( A[M2][N1],  A[M2][N2], -A[M2,N2])
    //     (-A[M2][N1], -A[M2][N2],  A[M2,N2])
    for (size_t y = 0; y < constraints.size(); y++) {
        for (const auto& i : N2) {
            constraints[y].emplace_back(-constraints[y][i]);
        }
    }
    for (size_t y = 0; y < M2.size(); y++) {
        constraints.emplace_back();
        constraints[y + original_constraints].reserve(total_vars);
        for (size_t x = 0; x < original_vars; x++) {
            constraints[y + original_constraints].emplace_back(-constraints[M2[y]][x]);
        }
        for (const auto& i : N2) {
            constraints[y + original_constraints].emplace_back(constraints[M2[y]][i]);
        }
    }

    std::ranges::fill(general.relations(), ">=");
    for (size_t i = 0; i < new_constraints; i++) {
        general.relations().emplace_back(">=");
    }

    std::ranges::fill(general.var_constraints(), ">=0");
    for (size_t i = 0; i < new_vars; i++) {
        general.var_constraints().emplace_back(">=0");
    }

    return general;
}

LinearProgram FormConverter::to_canonical_form(const LinearProgram& lp) {
    LinearProgram general = to_general_form(lp);

    auto& constraints = general.constraints();
    auto& rhs = general.rhs();

    const auto M1 = general.getM1();
    const auto M2 = general.getM2();
    const auto N1 = general.getN1();
    const auto N2 = general.getN2();

    size_t new_vars = N2.size() + M1.size();

    // c = (c[N1], c[N2], -c[N2], 0[M1])
    for (const auto& i : N2) {
        general.objective().emplace_back(-general.objective()[i]);
    }
    for (int i = 0; i < M1.size(); i++) {
        general.objective().emplace_back(0);
    }

    // A = (A[M1,N1], A[M1,N2], -A[M1,N2], -E[M1,M1]
    //     (A[M2,N1], A[M2,N2], -A[M2,N2],  0[M2,M1]
    for (size_t j = 0; j < M1.size(); j++) {
        const size_t y = M1[j];
        for (const auto& i : N2) {
            constraints[y].emplace_back(-general.constraints()[y][i]);
        }
        for (size_t i = 0; i < M1.size(); i++) {
            if (i == j) {
                constraints[y].emplace_back(-1);
            } else {
                constraints[y].emplace_back(0);
            }
        }
    }
    for (const auto& y : M2) {
        for (const auto& i : N2) {
            constraints[y].emplace_back(-general.constraints()[y][i]);
        }
        for (size_t i = 0; i < M1.size(); i++) {
            constraints[y].emplace_back(0);
        }
    }

    std::ranges::fill(general.var_constraints(), ">=0");
    for (size_t i = 0; i < new_vars; i++) {
        general.var_constraints().emplace_back(">=0");
    }
    // now new constraints
    std::ranges::fill(general.relations(), "=");

    // make rhs >= 0
    for (size_t i = 0; i < general.num_constraints(); ++i) {
        if (rhs[i] < 0) {
            for (size_t j = 0; j < general.num_variables(); ++j) {
                constraints[i][j] = -constraints[i][j];
            }
            rhs[i] = -rhs[i];
        }
    }

    general.prettierCoeffs();
    return general;
}
