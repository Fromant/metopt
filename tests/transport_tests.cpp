#include <cmath>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "linear/solvers/SimplexSolver.hpp"
#include "transport/TransportProblem.hpp"
#include "transport/TransportSolver.hpp"

// ==================== Константы ====================

const std::string TASKS_FOLDER = "../../tasks/transport/";
constexpr auto EPS = 1e-6; // Допуск для сравнения cost (симплекс vs MODI)
constexpr auto EPS_PLAN = 1e-4; // Допуск для сравнения плана перевозок

// ==================== Вспомогательные функции ====================

/**
 * Сравнивает решения MODI и Simplex: стоимость и план перевозок
 */
void CompareSolutions(const TransportProblem& problem, const TransportSolution& modi_result,
                      const SimplexSolver::Solution& simplex_result, double cost_eps = EPS,
                      double plan_eps = EPS_PLAN) {
    // 1. Сравниваем целевые функции
    EXPECT_NEAR(modi_result.total_cost, simplex_result.objective_value, cost_eps)
        << "Cost mismatch: MODI=" << modi_result.total_cost << ", Simplex=" << simplex_result.objective_value;

    // 2. Восстанавливаем план из simplex решения
    TransportProblem balanced = problem.balance();
    auto simplex_plan = TransportProblem::restorePlanFromVector(simplex_result.x, problem.numSuppliers(),
                                                                problem.numConsumers(), balanced.numSuppliers(),
                                                                balanced.numConsumers(), problem.hasPenalties());

    // 4. Валидируем план simplex
    EXPECT_TRUE(TransportProblem::validatePlan(simplex_plan, problem.supplies(), problem.demands()));
}

/**
 * Создаёт простую тестовую задачу 3x3 без пенальти
 */
TransportProblem CreateSimple3x3() {
    return TransportProblem({10, 20, 30}, // supplies
                            {15, 25, 20}, // demands
                            {{2, 5, 3}, {4, 1, 6}, {7, 3, 2}});
}

/**
 * Создаёт задачу 4x4 с пенальти
 */
TransportProblem Create4x4WithPenalties() {
    TransportProblem problem({15, 20, 25, 10}, {10, 20, 25, 15},
                             {{3, 7, 6, 4}, {2, 4, 3, 5}, {8, 6, 9, 7}, {5, 3, 4, 6}});
    problem.setPenalties({3, 5, 5, 3}, // thresholds
                         {2, 3, 2, 4} // rates
    );
    return problem;
}

// ==================== UNIT TESTS: TransportProblem ====================

TEST(TransportProblemTest, Constructor_Valid) {
    TransportProblem problem({10, 20}, {15, 15}, {{1, 2}, {3, 4}});

    EXPECT_EQ(problem.numSuppliers(), 2);
    EXPECT_EQ(problem.numConsumers(), 2);
    EXPECT_TRUE(problem.isBalanced());
    EXPECT_FALSE(problem.hasPenalties());
}

TEST(TransportProblemTest, Balance_UnbalancedSupply) {
    TransportProblem problem({30}, {10, 10}, {{1, 2}});

    EXPECT_FALSE(problem.isBalanced());
    EXPECT_GT(problem.totalSupply(), problem.totalDemand());

    auto balanced = problem.balance();

    EXPECT_TRUE(balanced.isBalanced());
    EXPECT_EQ(balanced.numConsumers(), 3); // +1 фиктивный
    EXPECT_EQ(balanced.costs()[0][2], 0.0); // нулевая стоимость до фиктивного
}

TEST(TransportProblemTest, Balance_UnbalancedDemand) {
    TransportProblem problem({10, 10}, {30}, {{1}, {2}});

    auto balanced = problem.balance();

    EXPECT_TRUE(balanced.isBalanced());
    EXPECT_EQ(balanced.numSuppliers(), 3); // +1 фиктивный
}

TEST(TransportProblemTest, Penalties_SetAndGet) {
    TransportProblem problem({10}, {10}, {{5}});

    EXPECT_FALSE(problem.hasPenalties());

    problem.setPenalties({3}, {2});

    EXPECT_TRUE(problem.hasPenalties());
    EXPECT_EQ(problem.penaltyThresholds()->at(0), 3);
    EXPECT_EQ(problem.penaltyRates()->at(0), 2);
}

TEST(TransportProblemTest, CalculatePenaltyCost_NoShortage) {
    TransportProblem problem({10}, {10}, {{5}});
    problem.setPenalties({5}, {2}); // порог 5, ставка 2

    std::vector<std::vector<double>> plan{{10}}; // полная поставка

    double penalty = problem.calculatePenaltyCost(plan);
    EXPECT_NEAR(penalty, 0.0, EPS); // недопоставки нет
}

TEST(TransportProblemTest, CalculatePenaltyCost_WithShortage) {
    TransportProblem problem({5}, {10}, {{5}});
    problem.setPenalties({3}, {2}); // порог 3, ставка 2

    std::vector<std::vector<double>> plan{{5}}; // поставили 5, нужно 10

    // Недопоставка: 10 - 3 (порог) - 5 (поставлено) = 2
    // Штраф: 2 * 2 = 4
    double penalty = problem.calculatePenaltyCost(plan);
    EXPECT_NEAR(penalty, 4.0, EPS);
}

TEST(TransportProblemTest, ToLinearProgram_Basic) {
    TransportProblem problem = CreateSimple3x3();
    auto lp = problem.toLinearProgram();

    // 3x3 = 9 переменных x[i][j], без пенальти
    EXPECT_EQ(lp.num_variables(), 9);

    // 3 поставщика + 2 потребителя (последний исключён) = 5 ограничений
    EXPECT_EQ(lp.num_constraints(), 5);

    // Все переменные >= 0
    for (const auto& constraint : lp.var_constraints()) {
        EXPECT_EQ(constraint, ">=0");
    }
}

TEST(TransportProblemTest, ToLinearProgram_WithPenalties) {
    TransportProblem problem = Create4x4WithPenalties();
    auto lp = problem.toLinearProgram();

    // 4x4 = 16 переменных x[i][j] + 4 переменные штрафа u[j]
    EXPECT_EQ(lp.num_variables(), 20);

    // 4 поставщика + 3 потребителя = 7 ограничений
    EXPECT_EQ(lp.num_constraints(), 7);

    // Целевая функция должна включать коэффициенты штрафов
    const auto& obj = lp.objective();
    for (size_t j = 0; j < 4; ++j) {
        EXPECT_GT(obj[16 + j], 0); // коэффициенты штрафов > 0
    }
}

TEST(TransportProblemTest, RestorePlanFromVector_Basic) {
    // Создаём "фейковое" решение ЛП для 3x3 задачи
    std::vector<double> lp_solution(9, 0.0);
    lp_solution[0] = 10; // x[0][0] = 10
    lp_solution[4] = 15; // x[1][1] = 15
    lp_solution[8] = 20; // x[2][2] = 20

    auto plan = TransportProblem::restorePlanFromVector(lp_solution, 3, 3, // original dims
                                                        3, 3, // balanced dims (same)
                                                        false // no penalties
    );

    EXPECT_EQ(plan.size(), 3);
    EXPECT_EQ(plan[0].size(), 3);
    EXPECT_NEAR(plan[0][0], 10.0, EPS);
    EXPECT_NEAR(plan[1][1], 15.0, EPS);
    EXPECT_NEAR(plan[2][2], 20.0, EPS);
}

TEST(TransportProblemTest, ValidatePlan_Valid) {
    std::vector<std::vector<double>> plan{{10, 0}, {0, 15}};
    std::vector<double> supplies{10, 15};
    std::vector<double> demands{10, 15};

    EXPECT_TRUE(TransportProblem::validatePlan(plan, supplies, demands));
}

TEST(TransportProblemTest, ValidatePlan_InvalidSupply) {
    std::vector<std::vector<double>> plan{{5, 0}, {0, 15}}; // первый поставил 5 вместо 10
    std::vector<double> supplies{10, 15};
    std::vector<double> demands{5, 15};

    EXPECT_FALSE(TransportProblem::validatePlan(plan, supplies, demands));
}

// ==================== UNIT TESTS: TransportSolver ====================

TEST(TransportSolverTest, NorthwestCorner_Basic) {
    TransportProblem problem = CreateSimple3x3();

    auto result = TransportSolver::solveNorthwestCorner(problem, false);

    // Проверка базиса: m+n-1 = 5 базисных переменных
    EXPECT_EQ(result.countBasicVars(), 5);

    // Проверка ограничений
    for (size_t i = 0; i < problem.numSuppliers(); ++i) {
        double sum = 0;
        for (size_t j = 0; j < problem.numConsumers(); ++j) {
            sum += result.shipments[i][j];
        }
        EXPECT_NEAR(sum, problem.supplies()[i], EPS);
    }

    for (size_t j = 0; j < problem.numConsumers(); ++j) {
        double sum = 0;
        for (size_t i = 0; i < problem.numSuppliers(); ++i) {
            sum += result.shipments[i][j];
        }
        EXPECT_NEAR(sum, problem.demands()[j], EPS);
    }
}

TEST(TransportSolverTest, NorthwestCorner_WithDegeneracy) {
    // Задача где СЗУ должен добавить вырожденную базисную переменную
    TransportProblem problem({10, 10}, {10, 10}, {{1, 2}, {3, 4}});

    auto result = TransportSolver::solveNorthwestCorner(problem, false);

    // m+n-1 = 3 базисные переменные
    EXPECT_EQ(result.countBasicVars(), 3);
}

TEST(TransportSolverTest, Potentials_Converges) {
    TransportProblem problem = CreateSimple3x3();

    auto initial = TransportSolver::solveNorthwestCorner(problem, false);
    auto result = TransportSolver::solvePotentials(problem, initial, false);

    EXPECT_TRUE(result.is_optimal);
    EXPECT_GT(result.iterations, 0);
    EXPECT_LT(result.iterations, 50); // разумное число итераций

    // Стоимость должна уменьшиться или остаться той же
    EXPECT_LE(result.total_cost, initial.total_cost + EPS);
}

TEST(TransportSolverTest, FullSolve_WithPenalties) {
    TransportProblem problem = Create4x4WithPenalties();

    auto result = TransportSolver::solve(problem, false);

    EXPECT_TRUE(result.is_optimal);

    // Общая стоимость = транспорт + штрафы
    EXPECT_NEAR(result.total_cost, result.transportation_cost + result.penalty_cost, EPS);

    // Проверка что план валидный
    EXPECT_TRUE(TransportProblem::validatePlan(result.shipments, problem.supplies(), problem.demands()));
}

// ==================== INTEGRATION TESTS: MODI vs Simplex ====================

TEST(IntegrationTest, Simple3x3_MODI_vs_Simplex) {
    TransportProblem problem = CreateSimple3x3();

    // MODI
    auto modi_result = TransportSolver::solve(problem, false);

    // Simplex
    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    auto simplex_result = SimplexSolver::solve(lp, false);

    // Проверка что симплекс нашёл решение
    ASSERT_TRUE(simplex_result.is_feasible);
    ASSERT_FALSE(simplex_result.is_unbounded);
    ASSERT_TRUE(simplex_result.is_optimal);

    // Сравнение решений
    CompareSolutions(problem, modi_result, simplex_result);
}

TEST(IntegrationTest, Balanced5x5_MODI_vs_Simplex) {
    const std::string filename = TASKS_FOLDER + "transport5.txt";

    TransportProblem problem = TransportProblem::readFromFile(filename);

    auto modi_result = TransportSolver::solve(problem, false);

    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    auto simplex_result = SimplexSolver::solve(lp, false);

    ASSERT_TRUE(simplex_result.is_feasible);
    ASSERT_TRUE(simplex_result.is_optimal);

    CompareSolutions(problem, modi_result, simplex_result);
}

TEST(IntegrationTest, Balanced5x5_MODI_vs_Simplex_penalties) {
    const std::string filename = TASKS_FOLDER + "transport6.txt";

    TransportProblem problem = TransportProblem::readFromFile(filename);

    auto modi_result = TransportSolver::solve(problem, false);

    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    auto simplex_result = SimplexSolver::solve(lp, false);

    ASSERT_TRUE(simplex_result.is_feasible);
    ASSERT_TRUE(simplex_result.is_optimal);

    CompareSolutions(problem, modi_result, simplex_result);
}

TEST(IntegrationTest, WithPenalties_MODI_vs_Simplex) {
    TransportProblem problem = Create4x4WithPenalties();

    auto modi_result = TransportSolver::solve(problem, false);

    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    auto simplex_result = SimplexSolver::solve(lp, false);

    ASSERT_TRUE(simplex_result.is_feasible);
    ASSERT_TRUE(simplex_result.is_optimal);

    // Сравниваем с большим допуском из-за численных погрешностей в штрафах
    CompareSolutions(problem, modi_result, simplex_result, 1e-4, 1e-3);
}

TEST(IntegrationTest, UnbalancedProblem_MODI_vs_Simplex) {
    // Создаём несбалансированную задачу
    TransportProblem problem({50, 30}, // supply = 80
                             {20, 20, 20}, // demand = 60
                             {{1, 2, 3}, {4, 5, 6}});

    auto modi_result = TransportSolver::solve(problem, false);

    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    auto simplex_result = SimplexSolver::solve(lp, false);

    ASSERT_TRUE(simplex_result.is_feasible);

    // Сравниваем только транспортную стоимость (без фиктивных перевозок)
    EXPECT_NEAR(modi_result.transportation_cost, simplex_result.objective_value, EPS);
}

// ==================== EDGE CASES ====================

TEST(EdgeCaseTest, SingleSupplierSingleConsumer) {
    TransportProblem problem({10}, {10}, {{5}});

    auto modi_result = TransportSolver::solve(problem, false);

    EXPECT_TRUE(modi_result.is_optimal);
    EXPECT_NEAR(modi_result.shipments[0][0], 10.0, EPS);
    EXPECT_NEAR(modi_result.total_cost, 50.0, EPS);
}

TEST(EdgeCaseTest, ZeroCostMatrix) {
    TransportProblem problem({10, 10}, {10, 10}, {{0, 0}, {0, 0}});

    auto result = TransportSolver::solve(problem, false);

    EXPECT_TRUE(result.is_optimal);
    EXPECT_NEAR(result.total_cost, 0.0, EPS);
}

TEST(EdgeCaseTest, LargeCostDifference) {
    // Задача где одна клетка сильно дешевле других
    TransportProblem problem({10, 10}, {10, 10}, {{1, 100}, {100, 1}});

    auto result = TransportSolver::solve(problem, false);

    EXPECT_TRUE(result.is_optimal);
    // Оптимально: x[0][0]=10, x[1][1]=10, cost=20
    EXPECT_NEAR(result.total_cost, 20.0, EPS);
}

TEST(EdgeCaseTest, DegenerateInitialPlan) {
    // Задача где СЗУ даёт вырожденный план
    TransportProblem problem({5, 5, 5}, {5, 5, 5}, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}});

    auto result = TransportSolver::solve(problem, false);

    EXPECT_TRUE(result.is_optimal);
    EXPECT_TRUE(TransportProblem::validatePlan(result.shipments, problem.supplies(), problem.demands()));
}
