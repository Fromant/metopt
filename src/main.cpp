#include "solver.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

using namespace bilp;

static void print_separator() {
    std::cout << std::string(60, '-') << "\n";
}

static void print_result(const std::string& name, const SolverResult& result,
                         const TestCase& tc) {
    print_separator();
    std::cout << "Test: " << name << "\n";
    print_separator();

    std::cout << "Feasible: " << (result.feasible ? "Yes" : "No") << "\n";
    std::cout << "Optimal NPV: " << std::fixed << std::setprecision(2)
              << result.optimal_npv << "\n";
    std::cout << "Nodes explored: " << result.nodes_explored << "\n";

    std::cout << "Selected items: ";
    bool first = true;
    for (int i = 0; i < tc.n; ++i) {
        if (result.solution[i] == 1) {
            if (!first) std::cout << ", ";
            std::cout << i;
            first = false;
        }
    }
    std::cout << "\n";

    std::cout << "Budget used: ";
    double total_cost = 0.0;
    for (int i = 0; i < tc.n; ++i) {
        if (result.solution[i] == 1) {
            total_cost += tc.costs[i];
        }
    }
    std::cout << std::fixed << std::setprecision(2) << total_cost
              << " / " << tc.budget << "\n";

    // Verify constraints
    bool all_ok = true;
    for (size_t j = 0; j < tc.constraints.A.size(); ++j) {
        double lhs = 0.0;
        for (int i = 0; i < tc.n; ++i) {
            lhs += tc.constraints.A[j][i] * result.solution[i];
        }
        bool ok = lhs <= tc.constraints.b[j] + 1e-6;
        if (!ok) {
            std::cout << "CONSTRAINT VIOLATION (row " << j << "): "
                      << std::fixed << std::setprecision(4) << lhs
                      << " > " << tc.constraints.b[j] << "\n";
            all_ok = false;
        }
    }
    std::cout << "Constraints: " << (all_ok ? "All satisfied" : "VIOLATED")
              << "\n";

    // Check against expected
    bool npv_match = std::abs(result.optimal_npv - tc.expected_npv) < 1e-6;
    std::cout << "Expected NPV: " << std::fixed << std::setprecision(2)
              << tc.expected_npv << " - "
              << (npv_match ? "MATCH" : "MISMATCH") << "\n";

    print_separator();
    std::cout << "\n";
}

int main() {
    std::cout << "=== Binary Integer Linear Programming Solver ===\n";
    std::cout << "    Branch & Bound with Constraint Propagation\n\n";

    // Run predefined test cases
    std::vector<TestCase> test_cases = {
        make_test_case_1(),
        make_test_case_2(),
    };

    int passed = 0;
    int total = static_cast<int>(test_cases.size());

    for (const auto& tc : test_cases) {
        SolverResult result;
        bool ok = run_test_case(tc, result);
        print_result(tc.name, result, tc);
        if (ok) {
            passed++;
        }
    }

    std::cout << "Summary: " << passed << " / " << total << " tests passed\n";

    return (passed == total) ? 0 : 1;
}
