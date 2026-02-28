#include <cstring>
#include <iostream>
#include <string>

#include "linear/solvers/SimplexSolver.hpp"
#include "transport/TransportProblem.hpp"
#include "transport/TransportSolver.hpp"

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options] [filename]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -v, --verbose      Enable verbose output (show cycles, potentials)" << std::endl;
    std::cout << "  -h, --help        Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "If no filename is provided, reads from console." << std::endl;
}

void solve_with_modi(const TransportProblem& problem, bool verbose) {
    std::cout << "\n========== MODI METHOD ==========" << std::endl;

    if (!problem.isBalanced()) {
        std::cout << "Problem is not balanced. Balancing..." << std::endl;
    }

    TransportProblem balanced = problem.balance();
    if (verbose) {
        balanced.print("Balanced Problem");
    }

    auto result = TransportSolver::solve(problem, verbose);

    result.print("Итоговый план");
}

void solve_with_simplex(const TransportProblem& original_problem, bool verbose) {
    std::cout << "\n========== SIMPLEX METHOD ==========" << std::endl;

    // 1. Балансируем задачу (как делает toLinearProgram)
    TransportProblem balanced = original_problem.balance();

    // 2. Преобразуем в ЛП
    LinearProgram lp = balanced.toLinearProgram();

    if (verbose) {
        lp.print("LP form of transport problem");
    }

    // 3. Решаем симплекс-методом
    auto result = SimplexSolver::solve(lp, false);

    if (!result.is_feasible) {
        std::cout << "ERROR: Problem is infeasible" << std::endl;
        return;
    }

    if (result.is_unbounded) {
        std::cout << "ERROR: Problem is unbounded" << std::endl;
        return;
    }

    if (!result.is_optimal) {
        std::cout << "WARNING: Solution is not optimal" << std::endl;
    }

    // 4. Восстанавливаем план ОРИГИНАЛЬНОЙ задачи
    auto plan = TransportProblem::restorePlanFromVector(
        result.x,  // вектор решения от симплекса
        original_problem.numSuppliers(),   // original_m
        original_problem.numConsumers(),   // original_n
        balanced.numSuppliers(),           // balanced_m
        balanced.numConsumers(),           // balanced_n
        balanced.penaltyRates().has_value() // has_penalties
    );

    // 5. Вывод результатов
    std::cout << "\n=== Simplex Solution ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Objective value: " << result.objective_value << "\n";
    std::cout << "Basis size: " << result.basis.size() << "\n";

    // Печатаем план перевозок
    TransportProblem::printPlan(
        plan,
        original_problem.supplies(),
        original_problem.demands(),
        "Optimal Transportation Plan (Simplex)"
    );

    // 6. Валидация: сравнение с методом потенциалов (опционально)
    if (verbose) {
        auto potentials_result = TransportSolver::solve(original_problem, false);
        std::cout << "\n=== Comparison ===" << std::endl;
        std::cout << "Simplex cost:    " << result.objective_value << "\n";
        std::cout << "Potentials cost: " << potentials_result.total_cost << "\n";
        std::cout << "Difference:      " << std::abs(result.objective_value - potentials_result.total_cost) << "\n";
    }
}

// void demonstrate_cycle(const TransportProblem& problem) {
//     std::cout << "\n========== CYCLE DEMONSTRATION ==========" << std::endl;
//
//     TransportProblem balanced = problem.balance();
//
//     auto nwResult = NorthwestCorner::solve(balanced, false);
//     if (!nwResult.success) {
//         std::cout << "ERROR: Failed to find initial plan" << std::endl;
//         return;
//     }
//
//     std::vector<double> u(balanced.m, 0.0);
//     std::vector<double> v(balanced.n, 0.0);
//     MODISolver::computePotentials(balanced, nwResult.basis, u, v);
//
//     std::cout << "\nPotentials:" << std::endl;
//     MODISolver::printPotentials(u, v);
//
//     std::cout << "\nDeltas (free cells):" << std::endl;
//     MODISolver::printDeltas(balanced, nwResult.basis, u, v);
//
//     auto enteringCell = MODISolver::findEnteringCell(balanced, nwResult.basis, u, v);
//     if (enteringCell) {
//         std::cout << "\nEntering cell: (A" << (enteringCell->first + 1)
//                   << ", B" << (enteringCell->second + 1) << ")" << std::endl;
//
//         auto cycle = MODISolver::findCycle(enteringCell->first, enteringCell->second, nwResult.basis, TODO, TODO);
//
//         std::cout << "\nCycle:" << std::endl;
//         MODISolver::printCycle(cycle, nwResult.plan);
//
//         std::cout << "\nApplying cycle..." << std::endl;
//         std::vector<std::vector<double>> newPlan = nwResult.plan;
//         size_t leavingIndex;
//         MODISolver::applyCycle(newPlan, cycle, leavingIndex);
//
//         std::cout << "After cycle (leaving basis cell index: " << leavingIndex << "):" << std::endl;
//         TransportProblem::printPlan(newPlan);
//         std::cout << "New cost: " << balanced.calculateCost(newPlan) << std::endl;
//     }
// }

int main(int argc, char* argv[]) {
    bool verbose = false;
    std::string filename;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            filename = argv[i];
        }
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "========================================" << std::endl;
    std::cout << "   Transport Problem Solver (Lab 2)" << std::endl;
    std::cout << "========================================" << std::endl;

    TransportProblem problem;

    if (!filename.empty()) {
        std::cout << "\nReading transport problem from file: " << filename << std::endl;
        problem = TransportProblem::readFromFile(filename);
        problem.print("Input Transport Problem");
    } else {
        std::cout << "\nNo input file specified. Reading from console..." << std::endl;
        problem = TransportProblem::readFromConsole();
    }
    std::cout << "Balanced: " << (problem.isBalanced() ? "Yes" : "No") << std::endl;

    solve_with_modi(problem, verbose);

    solve_with_simplex(problem, verbose);

    if (verbose) {
        // demonstrate_cycle(problem);
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Done!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
