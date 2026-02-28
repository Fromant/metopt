#include <cstring>
#include <iostream>
#include <string>

#include "lib.hpp"
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

    solve_with_modi(problem, true, verbose);

    solve_with_simplex(problem, true, verbose);

    if (verbose) {
        // demonstrate_cycle(problem);
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Done!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
