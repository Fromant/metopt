#include "solver.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <vector>

namespace bilp {

static constexpr double EPSILON = 1e-9;

// ====================================================================
// Solver implementation
// ====================================================================

Solver::Solver(const SolverConfig& config)
    : config_(config), nodes_explored_(0) {
    if (config_.n <= 0) {
        throw std::invalid_argument("Number of variables must be positive");
    }
    if (config_.budget < 0.0) {
        throw std::invalid_argument("Budget must be non-negative");
    }
    if (static_cast<int>(config_.costs.size()) != config_.n) {
        throw std::invalid_argument("Costs vector size mismatch");
    }
    if (static_cast<int>(config_.returns.size()) != config_.n) {
        throw std::invalid_argument("Returns vector size mismatch");
    }
    for (int i = 0; i < config_.n; ++i) {
        if (config_.costs[i] <= 0.0) {
            throw std::invalid_argument("All costs must be positive");
        }
        if (config_.returns[i] < 0.0) {
            throw std::invalid_argument("All returns must be non-negative");
        }
    }
}

SolverResult Solver::solve() {
    SolverResult result;
    result.feasible = false;
    result.optimal_npv = 0.0;
    result.nodes_explored = 0;
    result.solution.assign(config_.n, 0);

    double global_best_npv = 0.0;
    std::vector<int> global_best_assignment(config_.total_vars, 0);

    // Max-heap priority queue ordered by upper_bound (best-first)
    auto cmp = [](const Node& a, const Node& b) {
        // For equal upper_bounds, prefer deeper nodes (more constrained)
        if (std::abs(a.upper_bound - b.upper_bound) < EPSILON) {
            return a.depth < b.depth;
        }
        return a.upper_bound < b.upper_bound;
    };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> Q(cmp);

    // Initialize root node
    Node root;
    root.fixed.assign(config_.total_vars, -1);
    root.current_cost = 0.0;
    root.current_return = 0.0;
    root.depth = 0;
    root.upper_bound = compute_upper_bound(root);

    Q.push(std::move(root));

    while (!Q.empty()) {
        Node U = std::move(const_cast<Node&>(Q.top()));
        Q.pop();

        // Prune by bound
        if (U.upper_bound <= global_best_npv + EPSILON) {
            continue;
        }

        nodes_explored_++;

        // Constraint propagation
        if (!propagate_constraints(U)) {
            continue; // Infeasible after propagation
        }

        // Recompute bound after propagation
        U.upper_bound = compute_upper_bound(U);

        // Prune again after propagation
        if (U.upper_bound <= global_best_npv + EPSILON) {
            continue;
        }

        // Check if all variables are fixed
        if (all_fixed(U)) {
            if (U.current_return > global_best_npv + EPSILON) {
                global_best_npv = U.current_return;
                global_best_assignment = U.fixed;
            }
            continue;
        }

        // Select branching variable
        int k = select_branching_variable(U);
        if (k < 0) {
            continue; // Should not happen if not all_fixed
        }

        // Create children: branch on x[k] = 0 and x[k] = 1
        Node U0 = create_child(U, k, 0);
        Node U1 = create_child(U, k, 1);

        // Process U0
        if (U0.current_cost <= config_.budget + EPSILON) {
            U0.upper_bound = compute_upper_bound(U0);
            if (propagate_constraints(U0)) {
                U0.upper_bound = compute_upper_bound(U0);
                if (U0.upper_bound > global_best_npv + EPSILON) {
                    Q.push(std::move(U0));
                }
            }
        }

        // Process U1
        if (U1.current_cost <= config_.budget + EPSILON) {
            U1.upper_bound = compute_upper_bound(U1);
            if (propagate_constraints(U1)) {
                U1.upper_bound = compute_upper_bound(U1);
                if (U1.upper_bound > global_best_npv + EPSILON) {
                    Q.push(std::move(U1));
                }
            }
        }
    }

    result.feasible = true;
    result.optimal_npv = global_best_npv;
    result.nodes_explored = nodes_explored_;
    result.solution = extract_primary_solution(global_best_assignment);

    return result;
}

double Solver::compute_upper_bound(const Node& node) const {
    // Fractional relaxation: free variables can take values in [0, 1]
    double remaining_budget = config_.budget - node.current_cost;
    if (remaining_budget < 0.0) {
        return node.current_return; // Already infeasible, but return current
    }

    // Collect free variables with their efficiency
    struct FreeVar {
        int index;
        double efficiency; // r[i] / c[i]
        double cost;
        double return_val;
    };

    std::vector<FreeVar> free_vars;
    free_vars.reserve(config_.total_vars);

    for (int i = 0; i < config_.total_vars; ++i) {
        if (node.fixed[i] == -1) {
            // Only consider primary variables for relaxation (aux vars have 0 return)
            double cost = (i < config_.n) ? config_.costs[i] : 0.0;
            double ret = (i < config_.n) ? config_.returns[i] : 0.0;
            double eff = (cost > EPSILON) ? ret / cost : 0.0;
            free_vars.push_back({i, eff, cost, ret});
        }
    }

    // Sort by efficiency descending; break ties by index ascending (deterministic)
    std::sort(free_vars.begin(), free_vars.end(),
              [](const FreeVar& a, const FreeVar& b) {
                  if (std::abs(a.efficiency - b.efficiency) > EPSILON) {
                      return a.efficiency > b.efficiency;
                  }
                  return a.index < b.index;
              });

    double bound = node.current_return;
    double budget_left = remaining_budget;

    for (const auto& fv : free_vars) {
        if (budget_left <= EPSILON) {
            break;
        }
        if (fv.cost <= budget_left + EPSILON) {
            // Take the whole item
            bound += fv.return_val;
            budget_left -= fv.cost;
        } else {
            // Take fractionally
            bound += fv.efficiency * budget_left;
            budget_left = 0.0;
            break;
        }
    }

    return bound;
}

bool Solver::propagate_constraints(Node& node) const {
    // Iteratively propagate forced fixings from linear constraints
    // Returns false if a contradiction is detected
    bool changed = true;
    while (changed) {
        changed = false;

        for (size_t j = 0; j < config_.constraints.A.size(); ++j) {
            const auto& row = config_.constraints.A[j];
            double rhs = config_.constraints.b[j];

            // Collect contributions from fixed and free variables
            double fixed_lhs = 0.0; // sum of A[j][i] for vars fixed to 1
            std::vector<int> free_indices;
            double min_extra = 0.0; // min possible contribution from free vars

            for (size_t i = 0; i < row.size(); ++i) {
                if (i >= static_cast<size_t>(config_.total_vars)) break;

                double coeff = row[i];
                if (node.fixed[i] == -1) {
                    free_indices.push_back(static_cast<int>(i));
                    if (coeff < 0) {
                        min_extra += coeff; // negative coeff at x=1 gives min
                    }
                    // positive coeff at x=0 gives 0 contribution to min
                } else if (node.fixed[i] == 1) {
                    fixed_lhs += coeff;
                }
            }

            double max_extra = 0.0; // max possible contribution from free vars
            for (int f : free_indices) {
                double c = row[f];
                if (c > 0) {
                    max_extra += c; // positive coeff at x=1 gives max
                }
                // negative coeff at x=0 gives 0 contribution to max
            }

            double min_lhs = fixed_lhs + min_extra;
            double max_lhs = fixed_lhs + max_extra;

            // Constraint always satisfied: skip
            if (max_lhs <= rhs + EPSILON) {
                continue;
            }

            // Constraint can never be satisfied: infeasible node
            if (min_lhs > rhs + EPSILON) {
                return false;
            }

            // Try to force free variables
            for (int k : free_indices) {
                if (node.fixed[k] != -1) continue; // may have been fixed in this pass

                double coeff = row[k];

                // Compute min LHS with x[k]=1:
                // x[k]=1 contributes coeff, other free vars contribute their minimum
                double min_extra_without_k = min_extra - (coeff < 0 ? coeff : 0.0);
                double min_lhs_with_k1 = fixed_lhs + coeff + min_extra_without_k;

                if (min_lhs_with_k1 > rhs + EPSILON) {
                    // x[k]=1 is infeasible for this constraint, force x[k]=0
                    node.fixed[k] = 0;
                    changed = true;
                } else {
                    // Compute min LHS with x[k]=0:
                    // x[k]=0 contributes 0, other free vars contribute their minimum
                    double min_lhs_with_k0 = fixed_lhs + min_extra_without_k;

                    if (min_lhs_with_k0 > rhs + EPSILON) {
                        // x[k]=0 is infeasible for this constraint, force x[k]=1
                        node.fixed[k] = 1;
                        changed = true;

                        if (k < config_.n) {
                            node.current_cost += config_.costs[k];
                            node.current_return += config_.returns[k];
                        }
                    }
                }
            }
        }
    }

    // Final check: verify each constraint can still be satisfied
    // (min_lhs <= rhs means there exists some assignment of free vars that works)
    for (size_t j = 0; j < config_.constraints.A.size(); ++j) {
        const auto& row = config_.constraints.A[j];
        double rhs = config_.constraints.b[j];
        double fixed_lhs = 0.0;
        double min_extra = 0.0;

        for (size_t i = 0; i < row.size(); ++i) {
            if (i >= static_cast<size_t>(config_.total_vars)) break;
            double coeff = row[i];
            if (node.fixed[i] == -1) {
                if (coeff < 0) {
                    min_extra += coeff;
                }
            } else if (node.fixed[i] == 1) {
                fixed_lhs += coeff;
            }
        }

        double min_lhs = fixed_lhs + min_extra;
        if (min_lhs > rhs + EPSILON) {
            return false; // Contradiction: constraint can never be satisfied
        }
    }

    // Check budget constraint
    if (node.current_cost > config_.budget + EPSILON) {
        return false;
    }

    return true;
}

int Solver::select_branching_variable(const Node& node) const {
    // Select first free variable by index (deterministic)
    // Alternative: highest efficiency among free variables
    int best_idx = -1;

    for (int i = 0; i < config_.total_vars; ++i) {
        if (node.fixed[i] == -1) {
            if (best_idx < 0) {
                best_idx = i;
            }
        }
    }

    return best_idx;
}

Node Solver::create_child(const Node& parent, int k, int v) const {
    Node child = parent;
    child.fixed[k] = v;
    child.depth++;

    if (v == 1 && k < config_.n) {
        child.current_cost += config_.costs[k];
        child.current_return += config_.returns[k];
    }

    return child;
}

bool Solver::all_fixed(const Node& node) const {
    for (int i = 0; i < config_.total_vars; ++i) {
        if (node.fixed[i] == -1) {
            return false;
        }
    }
    return true;
}

std::vector<int> Solver::extract_primary_solution(const FixedState& full_state) const {
    std::vector<int> sol(config_.n, 0);
    for (int i = 0; i < config_.n; ++i) {
        if (full_state[i] == 1) {
            sol[i] = 1;
        }
    }
    return sol;
}

// ====================================================================
// Test case constraint builders
// ====================================================================

LinearConstraints build_test1_constraints() {
    // Test Case 1: n=7 (no auxiliary variables needed)
    // Constraint 1: x0 - x1 - x2 <= 0  (if x0=1 then x1+x2 >= 1)
    // Constraint 2: x0 + x1 + x2 <= 2  (at most 2 of first 3)
    // These two constraints together yield optimal NPV=65 with {0,2,4,5,6}

    LinearConstraints lc;
    lc.A.resize(2, std::vector<double>(7, 0.0));
    lc.b.resize(2);

    // Row 0: x0 - x1 - x2 <= 0
    lc.A[0][0] = 1.0;
    lc.A[0][1] = -1.0;
    lc.A[0][2] = -1.0;
    lc.b[0] = 0.0;

    // Row 1: x0 + x1 + x2 <= 2
    lc.A[1][0] = 1.0;
    lc.A[1][1] = 1.0;
    lc.A[1][2] = 1.0;
    lc.b[1] = 2.0;

    return lc;
}

LinearConstraints build_test2_constraints() {
    // Test Case 2: n=4
    // Constraint: x0 - x1 <= 0  (if x0=1 then x1=1)

    LinearConstraints lc;
    lc.A.resize(1, std::vector<double>(4, 0.0));
    lc.b.resize(1);

    lc.A[0][0] = 1.0;
    lc.A[0][1] = -1.0;
    lc.b[0] = 0.0;

    return lc;
}

TestCase make_test_case_1() {
    TestCase tc;
    tc.name = "Logical Constraints (7 vars, budget=20)";
    tc.n = 7;
    tc.budget = 20.0;
    tc.costs = {1.5, 2.5, 3.5, 6.0, 7.0, 4.5, 3.0};
    tc.returns = {16.0, 8.0, 10.0, 13.5, 22.0, 10.0, 7.0};
    tc.constraints = build_test1_constraints();
    tc.total_vars = 7;
    tc.expected_npv = 66.5; // Optimal: {0,1,3,4,6} cost=20, return=66.5
    tc.expected_selection = {0, 1, 3, 4, 6};
    return tc;
}

TestCase make_test_case_2() {
    TestCase tc;
    tc.name = "Implication Stress Test (4 vars, budget=10)";
    tc.n = 4;
    tc.budget = 10.0;
    tc.costs = {3.0, 4.0, 3.0, 4.0};
    tc.returns = {10.0, 12.0, 8.0, 11.0};
    tc.constraints = build_test2_constraints();
    tc.total_vars = 4;
    tc.expected_npv = 30.0; // Corrected: {0,1,2} gives cost=10, return=30
    tc.expected_selection = {0, 1, 2};
    return tc;
}

bool run_test_case(const TestCase& tc, SolverResult& result) {
    SolverConfig config;
    config.n = tc.n;
    config.budget = tc.budget;
    config.costs = tc.costs;
    config.returns = tc.returns;
    config.constraints = tc.constraints;
    config.total_vars = tc.total_vars;

    Solver solver(config);
    result = solver.solve();

    bool npv_ok = std::abs(result.optimal_npv - tc.expected_npv) < 1e-6;

    // Check selection match
    std::vector<int> actual_selection;
    for (int i = 0; i < tc.n; ++i) {
        if (result.solution[i] == 1) {
            actual_selection.push_back(i);
        }
    }

    bool selection_ok = (actual_selection == tc.expected_selection);

    return npv_ok && selection_ok && result.feasible;
}

} // namespace bilp
