#include "solver.hpp"
#include "logical_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <vector>
#include <sstream>

namespace bilp {

static constexpr double EPSILON = 1e-9;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  ASSERTION FAILED: " << msg \
                      << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

#define TEST_ASSERT_EQ(expected, actual, msg) \
    do { \
        if ((expected) != (actual)) { \
            std::cerr << "  ASSERTION FAILED: " << msg \
                      << " (expected=" << (expected) \
                      << ", actual=" << (actual) << ")" \
                      << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

#define TEST_ASSERT_NEAR(expected, actual, tol, msg) \
    do { \
        if (std::abs((expected) - (actual)) > (tol)) { \
            std::cerr << "  ASSERTION FAILED: " << msg \
                      << " (expected=" << (expected) \
                      << ", actual=" << (actual) << ")" \
                      << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

#define TEST_CASE(name) \
    static bool test_##name()

#define RUN_TEST(name) \
    do { \
        std::cout << "[RUN] " << #name << "\n"; \
        bool ok = test_##name(); \
        if (ok) { \
            std::cout << "[PASS] " << #name << "\n"; \
            passed++; \
        } else { \
            std::cout << "[FAIL] " << #name << "\n"; \
            failed++; \
        } \
        total++; \
    } while (0)

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

    auto cmp = [](const Node& a, const Node& b) {
        if (std::abs(a.upper_bound - b.upper_bound) < EPSILON) {
            return a.depth < b.depth;
        }
        return a.upper_bound < b.upper_bound;
    };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> Q(cmp);

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

        if (U.upper_bound <= global_best_npv + EPSILON) {
            continue;
        }

        nodes_explored_++;

        if (!propagate_constraints(U)) {
            continue;
        }

        U.upper_bound = compute_upper_bound(U);

        if (U.upper_bound <= global_best_npv + EPSILON) {
            continue;
        }

        if (all_fixed(U)) {
            if (U.current_return > global_best_npv + EPSILON) {
                global_best_npv = U.current_return;
                global_best_assignment = U.fixed;
            }
            continue;
        }

        int k = select_branching_variable(U);
        if (k < 0) {
            continue;
        }

        Node U0 = create_child(U, k, 0);
        Node U1 = create_child(U, k, 1);

        if (U0.current_cost <= config_.budget + EPSILON) {
            U0.upper_bound = compute_upper_bound(U0);
            if (propagate_constraints(U0)) {
                U0.upper_bound = compute_upper_bound(U0);
                if (U0.upper_bound > global_best_npv + EPSILON) {
                    Q.push(std::move(U0));
                }
            }
        }

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
    double remaining_budget = config_.budget - node.current_cost;
    if (remaining_budget < 0.0) {
        return node.current_return;
    }

    struct FreeVar {
        int index;
        double efficiency;
        double cost;
        double return_val;
    };

    std::vector<FreeVar> free_vars;
    free_vars.reserve(config_.total_vars);

    for (int i = 0; i < config_.total_vars; ++i) {
        if (node.fixed[i] == -1) {
            double cost = (i < config_.n) ? config_.costs[i] : 0.0;
            double ret = (i < config_.n) ? config_.returns[i] : 0.0;
            double eff = (cost > EPSILON) ? ret / cost : 0.0;
            free_vars.push_back({i, eff, cost, ret});
        }
    }

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
            bound += fv.return_val;
            budget_left -= fv.cost;
        } else {
            bound += fv.efficiency * budget_left;
            budget_left = 0.0;
            break;
        }
    }

    return bound;
}

bool Solver::propagate_constraints(Node& node) const {
    bool changed = true;
    while (changed) {
        changed = false;

        for (size_t j = 0; j < config_.constraints.A.size(); ++j) {
            const auto& row = config_.constraints.A[j];
            double rhs = config_.constraints.b[j];

            double fixed_lhs = 0.0;
            std::vector<int> free_indices;
            double min_extra = 0.0;

            for (size_t i = 0; i < row.size(); ++i) {
                if (i >= static_cast<size_t>(config_.total_vars)) break;

                double coeff = row[i];
                if (node.fixed[i] == -1) {
                    free_indices.push_back(static_cast<int>(i));
                    if (coeff < 0) {
                        min_extra += coeff;
                    }
                } else if (node.fixed[i] == 1) {
                    fixed_lhs += coeff;
                }
            }

            double max_extra = 0.0;
            for (int f : free_indices) {
                double c = row[f];
                if (c > 0) {
                    max_extra += c;
                }
            }

            double min_lhs = fixed_lhs + min_extra;
            double max_lhs = fixed_lhs + max_extra;

            if (max_lhs <= rhs + EPSILON) {
                continue;
            }

            if (min_lhs > rhs + EPSILON) {
                return false;
            }

            for (int k : free_indices) {
                if (node.fixed[k] != -1) continue;

                double coeff = row[k];
                double min_extra_without_k = min_extra - (coeff < 0 ? coeff : 0.0);
                double min_lhs_with_k1 = fixed_lhs + coeff + min_extra_without_k;

                if (min_lhs_with_k1 > rhs + EPSILON) {
                    node.fixed[k] = 0;
                    changed = true;
                } else {
                    double min_lhs_with_k0 = fixed_lhs + min_extra_without_k;

                    if (min_lhs_with_k0 > rhs + EPSILON) {
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
            return false;
        }
    }

    if (node.current_cost > config_.budget + EPSILON) {
        return false;
    }

    return true;
}

int Solver::select_branching_variable(const Node& node) const {
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
}