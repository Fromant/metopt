#ifndef LOGICAL_CONSTRAINTS_HPP
#define LOGICAL_CONSTRAINTS_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include "solver.hpp"

namespace bilp {
namespace logic {

struct LinearConstraint {
    std::vector<double> coeffs;
    double rhs;
    LinearConstraint(int total_vars = 0, double rhs_val = 0.0) 
        : coeffs(total_vars, 0.0), rhs(rhs_val) {}
};

inline LinearConstraint cnf_clause_to_linear(const std::vector<int>& clause, int total_vars) {
    LinearConstraint lc(total_vars, 0.0);
    double const_sum = 0.0;
    for (int lit : clause) {
        int idx = std::abs(lit) - 1;
        if (idx < 0 || idx >= total_vars) throw std::invalid_argument("Literal index out of range");
        if (lit > 0) {
            lc.coeffs[idx] = -1.0;
        } else {
            lc.coeffs[idx] = 1.0;
            const_sum += 1.0;
        }
    }
    lc.rhs = const_sum - 1.0;
    return lc;
}

inline std::vector<LinearConstraint> rule_implies_or(int antecedent, const std::vector<int>& consequents, int total_vars) {
    std::vector<int> clause;
    clause.push_back(-antecedent);
    for (int c : consequents) clause.push_back(c);
    return {cnf_clause_to_linear(clause, total_vars)};
}

struct ExactlyKEncoding {
    int aux_var_index;
    std::vector<LinearConstraint> constraints;
};

inline ExactlyKEncoding encode_exactly_k(const std::vector<int>& vars, int k, int aux_var, int total_vars, double big_M = -1.0) {
    if (big_M < 0) big_M = static_cast<double>(vars.size()) + 1.0;

    ExactlyKEncoding res;
    res.aux_var_index = aux_var;
    int aux_idx = aux_var - 1;
    double M = big_M;
    double k_d = static_cast<double>(k);

    // z = 1 <=> sum(vars) == k
    // Корректная Big-M формулировка (без противоречий):
    // 1. sum <= k + M*(1-z)
    // 2. sum >= k - M*(1-z)
    // 3. sum <= k - 1 + M*z + M
    // 4. sum >= k + 1 - M*z - M
    
    // 1
    LinearConstraint c1(total_vars, k_d + M);
    for (int v : vars) c1.coeffs[v - 1] = 1.0;
    c1.coeffs[aux_idx] = -M;
    res.constraints.push_back(c1);

    // 2
    LinearConstraint c2(total_vars, -k_d + M);
    for (int v : vars) c2.coeffs[v - 1] = -1.0;
    c2.coeffs[aux_idx] = -M;
    res.constraints.push_back(c2);

    // 3
    LinearConstraint c3(total_vars, k_d - 1.0 + M);
    for (int v : vars) c3.coeffs[v - 1] = 1.0;
    c3.coeffs[aux_idx] = -M;
    res.constraints.push_back(c3);

    // 4
    LinearConstraint c4(total_vars, -k_d - 1.0 + M);
    for (int v : vars) c4.coeffs[v - 1] = -1.0;
    c4.coeffs[aux_idx] = -M;
    res.constraints.push_back(c4);

    return res;
}

inline std::vector<LinearConstraint> rule_exactly_implies(
    const std::vector<int>& vars1, int k1,
    const std::vector<int>& vars2, int k2,
    int aux1_var, int aux2_var, int total_vars)
{
    std::vector<LinearConstraint> res;
    auto enc1 = encode_exactly_k(vars1, k1, aux1_var, total_vars);
    for (const auto& c : enc1.constraints) res.push_back(c);
    auto enc2 = encode_exactly_k(vars2, k2, aux2_var, total_vars);
    for (const auto& c : enc2.constraints) res.push_back(c);

    LinearConstraint impl(total_vars, 0.0);
    impl.coeffs[aux1_var - 1] = 1.0;
    impl.coeffs[aux2_var - 1] = -1.0;
    res.push_back(impl);
    return res;
}

inline void add_constraints(LinearConstraints& target, const std::vector<LinearConstraint>& new_c, int total_vars) {
    for (const auto& c : new_c) {
        if (static_cast<int>(c.coeffs.size()) != total_vars) throw std::invalid_argument("Constraint size mismatch");
        target.A.push_back(c.coeffs);
        target.b.push_back(c.rhs);
    }
}

} // namespace logic
} // namespace bilp
#endif