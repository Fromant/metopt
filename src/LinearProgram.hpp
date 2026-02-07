#pragma once

#include <algorithm>
#include <iomanip>
#include <string>
#include <vector>

/**
 * LinearProgram - represents a linear programming problem
 *
 * Example problem:
 *   minimize: 2*x1 + 3*x2 - x3
 *   subject to:
 *     x1 + 2*x2 + x3 >= 5    (constraint 1: a₁ = [1, 2, 1, 0, 0], b₁ = 5)
 *     3*x1 - x2     <= 4     (constraint 2: a₂ = [3, -1, 0, 0, 0], b₂ = 4)
 *     x1, x2 >= 0; x3 free   (variable constraints)
 *
 * Data structure:
 *   - minimize_: true for minimization, false for maximization
 *   - objective_: coefficients vector c = [c₁, c₂, c₃, c₄, c₅] for cᵀx
 *   - constraints_: matrix A where row i contains coefficients aᵢ = [aᵢ₁, aᵢ₂, ..., aᵢ₅]
 *   - relations_: constraint types ["<=", ">=", "="] for each row of A
 *   - rhs_: right-hand side vector b = [b₁, b₂, ..., bₘ]
 *   - var_constraints_: variable sign constraints [">=0", "<=0", "free"] for x₁..x₅
 */
class LinearProgram {
    bool minimize_; // Optimization direction: true = min, false = max
    std::vector<double> objective_; // Objective coefficients c (size = number of variables)
    std::vector<std::vector<double>> constraints_; // Constraint matrix A (m rows × n columns)
    std::vector<std::string> relations_; // Constraint types: "<=", ">=", "="
    std::vector<double> rhs_; // Right-hand side values b
    std::vector<std::string> var_constraints_; // Variable sign constraints

public:
    enum Form { GENERAL, SYMMETRIC, CANONIC, UNDEFINED };

    // Constructors
    LinearProgram();
    LinearProgram(bool minimize, std::vector<double> objective, std::vector<std::vector<double>> constraints,
                  std::vector<std::string> relations, std::vector<double> rhs,
                  std::vector<std::string> var_constraints);

    // Input methods
    static LinearProgram read_from_file(const std::string& filename);
    static LinearProgram read_from_console();

    // Validation helpers
    static bool is_valid_constraint_type(const std::string& rel);
    static bool is_valid_variable_constraint(const std::string& constraint);

    // Accessors
    auto num_constraints() const { return constraints_.size(); }
    auto num_variables() const { return objective_.size(); }

    auto& is_minimization() { return minimize_; }
    auto& objective() { return objective_; }
    auto& constraints() { return constraints_; }
    auto& relations() { return relations_; }
    auto& rhs() { return rhs_; }
    auto& var_constraints() { return var_constraints_; }

    // const accessors
    const auto& is_minimization() const { return minimize_; }
    const auto& objective() const { return objective_; }
    const auto& constraints() const { return constraints_; }
    const auto& relations() const { return relations_; }
    const auto& rhs() const { return rhs_; }
    const auto& var_constraints() const { return var_constraints_; }

    Form getForm() const {
        if (!minimize_) {
            return UNDEFINED;
        }
        const bool isAllVarConstraintsGE =
            std::ranges::all_of(var_constraints(), [](const std::string& f) { return f == ">=0"; });
        const bool isAllConstraintsGE =
            std::ranges::all_of(relations(), [](const std::string& f) { return f == ">="; });

        if (isAllVarConstraintsGE && isAllConstraintsGE) {
            return SYMMETRIC;
        }

        const bool isAllConstraintsE = std::ranges::all_of(relations(), [](const std::string& f) { return f == "="; });
        if (isAllConstraintsE && isAllVarConstraintsGE) {
            return CANONIC;
        }

        const bool isAllConstraintsEorGE =
            std::ranges::all_of(relations(), [](const std::string& f) { return f == "=" || f == ">="; });
        const bool isAllVarConstraintsGEorFree =
            std::ranges::all_of(var_constraints(), [](const std::string& f) { return f == "free" || f==">=0"; });

        if (isAllConstraintsEorGE && isAllVarConstraintsGEorFree) {
            return GENERAL;
        }

        return UNDEFINED;
    }

    /// A[M1, N] * x[N] >= b[N]
    /// @return M1 - set of indices
    std::vector<size_t> getM1() const {
        std::vector<size_t> tr;
        for (size_t i = 0; i < relations_.size(); i++) {
            if (relations_[i] == ">=") {
                tr.emplace_back(i);
            }
        }
        return tr;
    }

    /// A[M1, N] * x[N] = b[N]
    /// @return M2 - set of indices
    std::vector<size_t> getM2() const {
        std::vector<size_t> tr;
        for (size_t i = 0; i < relations_.size(); i++) {
            if (relations_[i] == "=") {
                tr.emplace_back(i);
            }
        }
        return tr;
    }

    /// x[N1] >= 0
    /// @return N1 - set of indices
    std::vector<size_t> getN1() const {
        std::vector<size_t> tr;
        for (size_t i = 0; i < var_constraints_.size(); i++) {
            if (var_constraints_[i] == ">=0") {
                tr.emplace_back(i);
            }
        }
        return tr;
    }
    /// N2 = N/N1, x[N1]>=0
    /// @return N2 - set of indices
    std::vector<size_t> getN2() const {
        std::vector<size_t> tr;
        for (size_t i = 0; i < var_constraints_.size(); i++) {
            if (var_constraints_[i] != ">=0") {
                tr.emplace_back(i);
            }
        }
        return tr;
    }

    void prettierCoeffs() {
        for (int i = 0; i < constraints_.size(); i++) {
            if (relations_[i] == "=" && rhs_[i] <= 0.0) {
                for (double& j : constraints_[i]) {
                    j = -j;
                }
                rhs_[i] = -rhs_[i];
            }
        }
    }

    void print(const std::string& title) const;
};
