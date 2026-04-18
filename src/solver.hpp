#ifndef BILP_SOLVER_HPP
#define BILP_SOLVER_HPP

#include <vector>
#include <optional>
#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <numeric>
#include <iomanip>
#include <iostream>

namespace bilp {

// Fixed variable state: -1 = free, 0 = fixed to 0, 1 = fixed to 1
using FixedState = std::vector<int>;

// Constraint matrix: A * x <= b
struct LinearConstraints {
    std::vector<std::vector<double>> A; // m x n matrix
    std::vector<double> b;              // rhs vector, size m
};

// Result of solving a BILP instance
struct SolverResult {
    bool feasible;
    double optimal_npv;
    std::vector<int> solution; // x[i] in {0, 1}, size n
    int nodes_explored;
};

// Internal node representation for the B&B tree
struct Node {
    FixedState fixed;        // -1=free, 0=0, 1=1
    double current_cost;     // sum of c[i]*x[i] for fixed-to-1 vars
    double current_return;   // sum of r[i]*x[i] for fixed-to-1 vars
    double upper_bound;      // fractional relaxation upper bound
    int depth;               // number of fixed variables

    // Priority queue orders by upper_bound descending (best-first)
    bool operator<(const Node& other) const {
        return upper_bound < other.upper_bound;
    }
};

// Configuration for the solver
struct SolverConfig {
    int n;                          // number of primary variables
    double budget;                  // budget constraint B
    std::vector<double> costs;      // c[i], size n
    std::vector<double> returns;    // r[i], size n
    LinearConstraints constraints;  // A*x <= b (includes auxiliary vars if any)
    int total_vars;                 // n + number of auxiliary variables
    std::vector<int> primary_vars;  // indices of primary variables (0..n-1)
    std::vector<int> aux_vars;      // indices of auxiliary variables
};

class Solver {
public:
    explicit Solver(const SolverConfig& config);

    // Solve the BILP instance and return the result
    SolverResult solve();

    // Get the number of nodes explored (for diagnostics)
    int get_nodes_explored() const { return nodes_explored_; }

private:
    SolverConfig config_;
    int nodes_explored_;

    // Compute fractional relaxation upper bound for a node
    double compute_upper_bound(const Node& node) const;

    // Constraint propagation: deduce forced fixings from linear constraints
    // Returns false if contradiction detected (node is infeasible)
    bool propagate_constraints(Node& node) const;

    // Select branching variable from free variables
    // Returns index of variable to branch on, or -1 if none free
    int select_branching_variable(const Node& node) const;

    // Create a child node by fixing variable k to value v (0 or 1)
    Node create_child(const Node& parent, int k, int v) const;

    // Check if all variables are fixed
    bool all_fixed(const Node& node) const;

    // Extract primary variable solution from full assignment
    std::vector<int> extract_primary_solution(const FixedState& full_state) const;
};

// ============================================================
// Helper: build linear constraints from logical rules
// ============================================================

// Build constraints for Test Case 1:
// Logical rules:
//   R1: if x0 then (x1 or x2)
//   R2: exactly one of {x0,x1,x2} is selected  (sum == 1)
//   R3: exactly two of {x3,x4,x5} are selected (sum == 2)
// Uses auxiliary variable z = indicator(sum{x0,x1,x2} == 1)
// Returns LinearConstraints and total_vars (n + 1 auxiliary)
LinearConstraints build_test1_constraints();

// Build constraints for Test Case 2:
// Logical rule: if x0 then x1  =>  x0 - x1 <= 0
LinearConstraints build_test2_constraints();

// ============================================================
// Test runner utilities
// ============================================================

struct TestCase {
    std::string name;
    int n;
    double budget;
    std::vector<double> costs;
    std::vector<double> returns;
    LinearConstraints constraints;
    int total_vars;
    std::vector<int> aux_var_indices;
    double expected_npv;
    std::vector<int> expected_selection; // 0-based indices of selected primary vars
};

TestCase make_test_case_1();
TestCase make_test_case_2();

bool run_test_case(const TestCase& tc, SolverResult& result);

} // namespace bilp

#endif // BILP_SOLVER_HPP
