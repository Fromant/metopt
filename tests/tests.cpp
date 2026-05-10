#include "solver.hpp"
#include "logical_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

using namespace bilp;

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

static LinearConstraints build_test1_constraints() {
    LinearConstraints lc;
    lc.A.resize(2, std::vector<double>(7, 0.0));
    lc.b.resize(2);
    lc.A[0][0] = 1.0; lc.A[0][1] = -1.0; lc.A[0][2] = -1.0; lc.b[0] = 0.0;
    lc.A[1][0] = 1.0; lc.A[1][1] = 1.0;  lc.A[1][2] = 1.0;  lc.b[1] = 2.0;
    return lc;
}

static LinearConstraints build_test2_constraints() {
    LinearConstraints lc;
    lc.A.resize(1, std::vector<double>(4, 0.0));
    lc.b.resize(1);
    lc.A[0][0] = 1.0; lc.A[0][1] = -1.0; lc.b[0] = 0.0;
    return lc;
}

static TestCase make_test_case_1() {
    TestCase tc;
    tc.name = "Logical Constraints (7 vars, budget=20)";
    tc.n = 7; tc.budget = 20.0;
    tc.costs = {1.5, 2.5, 3.5, 6.0, 7.0, 4.5, 3.0};
    tc.returns = {16.0, 8.0, 10.0, 13.5, 22.0, 10.0, 7.0};
    tc.constraints = build_test1_constraints();
    tc.total_vars = 7;
    tc.expected_npv = 66.5;
    tc.expected_selection = {0, 1, 3, 4, 6};
    return tc;
}

static TestCase make_test_case_2() {
    TestCase tc;
    tc.name = "Implication Stress Test (4 vars, budget=10)";
    tc.n = 4; tc.budget = 10.0;
    tc.costs = {3.0, 4.0, 3.0, 4.0};
    tc.returns = {10.0, 12.0, 8.0, 11.0};
    tc.constraints = build_test2_constraints();
    tc.total_vars = 4;
    tc.expected_npv = 30.0;
    tc.expected_selection = {0, 1, 2};
    return tc;
}

static bool run_test_case(const TestCase& tc, SolverResult& result) {
    SolverConfig config;
    config.n = tc.n; config.budget = tc.budget;
    config.costs = tc.costs; config.returns = tc.returns;
    config.constraints = tc.constraints; config.total_vars = tc.total_vars;
    Solver solver(config);
    result = solver.solve();
    bool npv_ok = std::abs(result.optimal_npv - tc.expected_npv) < 1e-6;
    std::vector<int> actual_selection;
    for (int i = 0; i < tc.n; ++i) {
        if (result.solution[i] == 1) actual_selection.push_back(i);
    }
    bool selection_ok = (actual_selection == tc.expected_selection);
    return npv_ok && selection_ok && result.feasible;
}

TEST_CASE(test_implies_or) {
    using namespace bilp::logic;
    const int N = 3, TOTAL = N;
    SolverConfig config;
    config.n = N; config.budget = 10.0;
    config.costs = {2.0, 3.0, 4.0}; config.returns = {8.0, 10.0, 12.0};
    config.total_vars = TOTAL;
    config.primary_vars.resize(N); std::iota(config.primary_vars.begin(), config.primary_vars.end(), 0);
    
    LinearConstraints lc;
    auto rule = rule_implies_or(1, {2, 3}, TOTAL);
    add_constraints(lc, rule, TOTAL);
    config.constraints = lc;
    
    Solver solver(config);
    SolverResult result = solver.solve();
    
    TEST_ASSERT(result.feasible, "Solution must be feasible");
    TEST_ASSERT_NEAR(result.optimal_npv, 30.0, 1e-6, "Optimal NPV should be 30.0");
    if (result.solution[0] == 1) {
        TEST_ASSERT(result.solution[1] == 1 || result.solution[2] == 1,
                    "If x0=1 then x1 or x2 must be 1");
    }
    return true;
}

TEST_CASE(test_exactly_implies_exactly) {
    using namespace bilp::logic;
    const int PRIMARY_N = 6, AUX1 = 7, AUX2 = 8, TOTAL_VARS = PRIMARY_N + 2;
    SolverConfig config;
    config.n = PRIMARY_N; config.budget = 30.0;
    config.costs = {3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    config.returns = {10.0, 12.0, 15.0, 18.0, 20.0, 22.0};
    config.total_vars = TOTAL_VARS;
    config.primary_vars.resize(PRIMARY_N); std::iota(config.primary_vars.begin(), config.primary_vars.end(), 0);
    config.aux_vars = {AUX1 - 1, AUX2 - 1};
    
    LinearConstraints lc;
    auto rule = rule_exactly_implies({1, 2, 3}, 1, {4, 5, 6}, 2, AUX1, AUX2, TOTAL_VARS);
    add_constraints(lc, rule, TOTAL_VARS);
    config.constraints = lc;
    
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Solution must be feasible");
    
    int sum_first = result.solution[0] + result.solution[1] + result.solution[2];
    int sum_second = result.solution[3] + result.solution[4] + result.solution[5];
    if (sum_first == 1) {
        TEST_ASSERT_EQ(2, sum_second, "If exactly 1 from first group, then exactly 2 from second");
    }
    return true;
}

TEST_CASE(test_combined_rules) {
    using namespace bilp::logic;
    const int PRIMARY_N = 7, AUX1 = 8, AUX2 = 9, TOTAL_VARS = PRIMARY_N + 2;
    SolverConfig config;
    config.n = PRIMARY_N; config.budget = 25.0;
    config.costs = {3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 4.0};
    config.returns = {10.0, 12.0, 15.0, 18.0, 20.0, 22.0, 11.0};
    config.total_vars = TOTAL_VARS;
    config.primary_vars.resize(PRIMARY_N); std::iota(config.primary_vars.begin(), config.primary_vars.end(), 0);
    config.aux_vars = {AUX1 - 1, AUX2 - 1};
    
    LinearConstraints lc;
    auto rule1 = rule_implies_or(1, {2, 3}, TOTAL_VARS);
    add_constraints(lc, rule1, TOTAL_VARS);
    auto rule2 = rule_exactly_implies({1, 2, 3}, 1, {4, 5, 6}, 2, AUX1, AUX2, TOTAL_VARS);
    add_constraints(lc, rule2, TOTAL_VARS);
    config.constraints = lc;
    
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Solution must be feasible");
    
    if (result.solution[0] == 1) {
        TEST_ASSERT(result.solution[1] == 1 || result.solution[2] == 1, "Rule 1 violated");
    }
    int sum_first = result.solution[0] + result.solution[1] + result.solution[2];
    int sum_second = result.solution[3] + result.solution[4] + result.solution[5];
    if (sum_first == 1) {
        TEST_ASSERT_EQ(2, sum_second, "Rule 2 violated");
    }
    return true;
}

TEST_CASE(test_upper_bound_validity) {
    SolverConfig config;
    config.n = 3; config.budget = 5.0;
    config.costs = {2.0, 3.0, 4.0}; config.returns = {10.0, 12.0, 15.0};
    config.total_vars = 3; config.primary_vars = {0, 1, 2}; config.aux_vars = {};
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Simple case should be feasible");
    TEST_ASSERT_NEAR(result.optimal_npv, 22.0, 1e-6, "Optimal for simple case should be 22.0");
    return true;
}

TEST_CASE(test_budget_too_tight) {
    SolverConfig config;
    config.n = 3; config.budget = 0.5;
    config.costs = {2.0, 3.0, 4.0}; config.returns = {10.0, 12.0, 15.0};
    config.total_vars = 3; config.primary_vars = {0, 1, 2}; config.aux_vars = {};
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Empty selection is always feasible");
    TEST_ASSERT_NEAR(result.optimal_npv, 0.0, 1e-6, "With tight budget, optimal is empty set");
    return true;
}

TEST_CASE(test_propagation_forces_assignment) {
    SolverConfig config;
    config.n = 2; config.budget = 100.0;
    config.costs = {1.0, 1.0}; config.returns = {5.0, 10.0};
    config.total_vars = 2; config.primary_vars = {0, 1}; config.aux_vars = {};
    config.constraints.A = {{-1.0, -1.0}}; config.constraints.b = {-1.0};
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Should be feasible");
    TEST_ASSERT_NEAR(result.optimal_npv, 15.0, 1e-6, "Both items should be selected for max NPV");
    return true;
}

TEST_CASE(test_single_variable) {
    SolverConfig config;
    config.n = 1; config.budget = 5.0;
    config.costs = {3.0}; config.returns = {10.0};
    config.total_vars = 1; config.primary_vars = {0}; config.aux_vars = {};
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Single var should be feasible");
    TEST_ASSERT_NEAR(result.optimal_npv, 10.0, 1e-6, "Single var with positive return");
    TEST_ASSERT_EQ(1, result.solution[0], "Should select the single item");
    return true;
}

TEST_CASE(test_deterministic_output) {
    using namespace bilp::logic;
    const int N = 3, TOTAL = N;
    auto make_config = [&]() {
        SolverConfig cfg;
        cfg.n = N; cfg.budget = 10.0;
        cfg.costs = {2.0, 3.0, 4.0}; cfg.returns = {8.0, 10.0, 12.0};
        cfg.total_vars = TOTAL; cfg.primary_vars.resize(N);
        std::iota(cfg.primary_vars.begin(), cfg.primary_vars.end(), 0);
        cfg.aux_vars = {};
        LinearConstraints lc;
        auto rule = rule_implies_or(1, {2, 3}, TOTAL);
        add_constraints(lc, rule, TOTAL);
        cfg.constraints = lc;
        return cfg;
    };
    Solver s1(make_config()), s2(make_config());
    SolverResult r1 = s1.solve(), r2 = s2.solve();
    TEST_ASSERT_NEAR(r1.optimal_npv, r2.optimal_npv, 1e-9, "NPV mismatch across runs");
    TEST_ASSERT(r1.solution == r2.solution, "Solutions mismatch across runs");
    return true;
}

TEST_CASE(test_contradictory_constraints) {
    SolverConfig config;
    config.n = 2; config.budget = 10.0;
    config.costs = {1.0, 1.0}; config.returns = {5.0, 5.0};
    config.total_vars = 2; config.primary_vars = {0, 1}; config.aux_vars = {};
    config.constraints.A = {{1.0, 0.0}, {-1.0, 0.0}}; config.constraints.b = {0.0, -1.0};
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Solver should return feasible (possibly empty)");
    return true;
}

TEST_CASE(test_aux_vars_binary) {
    using namespace bilp::logic;
    const int PRIMARY_N = 4, AUX = 5, TOTAL_VARS = PRIMARY_N + 1;
    SolverConfig config;
    config.n = PRIMARY_N; config.budget = 20.0;
    config.costs = {2.0, 3.0, 4.0, 5.0}; config.returns = {8.0, 10.0, 12.0, 15.0};
    config.total_vars = TOTAL_VARS; config.primary_vars.resize(PRIMARY_N);
    std::iota(config.primary_vars.begin(), config.primary_vars.end(), 0);
    config.aux_vars = {AUX - 1};
    LinearConstraints lc;
    auto enc = encode_exactly_k({1, 2, 3}, 2, AUX, TOTAL_VARS);
    for (const auto& c : enc.constraints) {
        lc.A.push_back(c.coeffs); lc.b.push_back(c.rhs);
    }
    config.constraints = lc;
    Solver solver(config);
    SolverResult result = solver.solve();
    TEST_ASSERT(result.feasible, "Should be feasible with aux var encoding");
    int sum = result.solution[0] + result.solution[1] + result.solution[2];
    TEST_ASSERT(sum >= 0 && sum <= 3, "Sum of first 3 vars must be in [0,3]");
    return true;
}

TEST_CASE(test_original_case1) {
    auto tc = make_test_case_1(); SolverResult result;
    TEST_ASSERT(run_test_case(tc, result), "Test case 1 failed");
    TEST_ASSERT_NEAR(result.optimal_npv, 66.5, 1e-6, "NPV mismatch");
    return true;
}

TEST_CASE(test_original_case2) {
    auto tc = make_test_case_2(); SolverResult result;
    TEST_ASSERT(run_test_case(tc, result), "Test case 2 failed");
    TEST_ASSERT_NEAR(result.optimal_npv, 30.0, 1e-6, "NPV mismatch");
    return true;
}

int run_all_tests() {
    std::cout << "=== BILP Solver Test Suite ===\n\n";
    int passed = 0, failed = 0, total = 0;
    RUN_TEST(test_implies_or);
    RUN_TEST(test_exactly_implies_exactly);
    RUN_TEST(test_combined_rules);
    RUN_TEST(test_upper_bound_validity);
    RUN_TEST(test_budget_too_tight);
    RUN_TEST(test_propagation_forces_assignment);
    RUN_TEST(test_single_variable);
    RUN_TEST(test_deterministic_output);
    RUN_TEST(test_contradictory_constraints);
    RUN_TEST(test_aux_vars_binary);
    RUN_TEST(test_original_case1);
    RUN_TEST(test_original_case2);
    std::cout << "\n===================================\n";
    std::cout << "Results: " << passed << " passed, " << failed << " failed, " << total << " total\n";
    return failed > 0 ? 1 : 0;
}

int main() {
    return run_all_tests();
}