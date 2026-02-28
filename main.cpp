#include <iostream>
#include <string>
#include <cstring>
#include <numeric>

#include "transport/TransportProblem.hpp"
#include "transport/NorthwestCorner.hpp"
#include "transport/MODISolver.hpp"
#include "linear/solvers/SimplexSolver.hpp"

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
    
    auto nwResult = NorthwestCorner::solve(balanced, verbose);
    if (!nwResult.success) {
        std::cout << "ERROR: Failed to find initial plan with Northwest Corner method" << std::endl;
        std::cout << nwResult.errorMessage << std::endl;
        return;
    }
    
    if (verbose) {
        std::cout << "\nInitial plan (Northwest Corner):" << std::endl;
        TransportProblem::printPlan(nwResult.plan);
        std::cout << "Initial cost: " << balanced.calculateCost(nwResult.plan) << std::endl;
    }
    
    auto modiResult = MODISolver::solve(balanced, nwResult.plan, nwResult.basis, verbose);
    
    if (!modiResult.success) {
        std::cout << "ERROR: MODI solver failed: " << modiResult.errorMessage << std::endl;
        return;
    }
    
    std::cout << "\nMODI Solution:" << std::endl;
    std::cout << "  Iterations: " << modiResult.iterations << std::endl;
    std::cout << "  Optimal: " << (modiResult.isOptimal ? "Yes" : "No") << std::endl;
    std::cout << "  Total cost: " << modiResult.totalCost << std::endl;
    std::cout << "  Basis size: " << modiResult.basis.size() << std::endl;
    std::cout << "\nOptimal plan:" << std::endl;
    TransportProblem::printPlan(modiResult.plan);
}

void solve_with_simplex(const TransportProblem& problem, bool verbose) {
    std::cout << "\n========== SIMPLEX METHOD ==========" << std::endl;
    
    TransportProblem balanced = problem.balance();
    LinearProgram lp = balanced.toLinearProgram();
    
    if (verbose) {
        lp.print("LP Formulation");
    }
    
    auto result = SimplexSolver::solve(lp, verbose);
    
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
    
    auto plan = TransportProblem::restorePlanFromVector(result.x, balanced.m, balanced.n);
    
    std::cout << "\nSimplex Solution:" << std::endl;
    std::cout << "  Objective value: " << result.objective_value << std::endl;
    std::cout << "  Basis size: " << result.basis.size() << std::endl;
    std::cout << "\nOptimal plan:" << std::endl;
    TransportProblem::printPlan(plan);
}

void demonstrate_cycle(const TransportProblem& problem) {
    std::cout << "\n========== CYCLE DEMONSTRATION ==========" << std::endl;
    
    TransportProblem balanced = problem.balance();
    
    auto nwResult = NorthwestCorner::solve(balanced, false);
    if (!nwResult.success) {
        std::cout << "ERROR: Failed to find initial plan" << std::endl;
        return;
    }
    
    std::vector<double> u(balanced.m, 0.0);
    std::vector<double> v(balanced.n, 0.0);
    MODISolver::computePotentials(balanced, nwResult.basis, u, v);
    
    std::cout << "\nPotentials:" << std::endl;
    MODISolver::printPotentials(u, v);
    
    std::cout << "\nDeltas (free cells):" << std::endl;
    MODISolver::printDeltas(balanced, nwResult.basis, u, v);
    
    auto enteringCell = MODISolver::findEnteringCell(balanced, nwResult.basis, u, v);
    if (enteringCell) {
        std::cout << "\nEntering cell: (A" << (enteringCell->first + 1) 
                  << ", B" << (enteringCell->second + 1) << ")" << std::endl;
        
        auto cycle = MODISolver::findCycle(enteringCell->first, enteringCell->second, nwResult.basis);
        
        std::cout << "\nCycle:" << std::endl;
        MODISolver::printCycle(cycle, nwResult.plan);
        
        std::cout << "\nApplying cycle..." << std::endl;
        std::vector<std::vector<double>> newPlan = nwResult.plan;
        size_t leavingIndex;
        MODISolver::applyCycle(newPlan, cycle, leavingIndex);
        
        std::cout << "After cycle (leaving basis cell index: " << leavingIndex << "):" << std::endl;
        TransportProblem::printPlan(newPlan);
        std::cout << "New cost: " << balanced.calculateCost(newPlan) << std::endl;
    }
}

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
        problem = TransportProblem::fromFile(filename);
        problem.print("Input Transport Problem");
        
        if (!problem.demandPenalty.empty()) {
            std::cout << "\nDemand penalties: ";
            for (size_t j = 0; j < problem.demandPenalty.size(); ++j) {
                std::cout << problem.demandPenalty[j] << " ";
            }
            std::cout << std::endl;
        }
        
        if (!problem.demandThreshold.empty()) {
            std::cout << "\nDemand thresholds: ";
            for (size_t j = 0; j < problem.demandThreshold.size(); ++j) {
                std::cout << problem.demandThreshold[j] << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "\nNo input file specified. Reading from console..." << std::endl;
        problem = TransportProblem::fromConsole();
    }
    
    if (!problem.validate()) {
        std::cout << "ERROR: Invalid transport problem" << std::endl;
        return 1;
    }
    
    std::cout << "\nTotal supply: " << std::accumulate(problem.supply.begin(), problem.supply.end(), 0.0) << std::endl;
    std::cout << "Total demand: " << std::accumulate(problem.demand.begin(), problem.demand.end(), 0.0) << std::endl;
    std::cout << "Balanced: " << (problem.isBalanced() ? "Yes" : "No") << std::endl;
    
    solve_with_modi(problem, verbose);
    
    solve_with_simplex(problem, verbose);
    
    if (verbose) {
        demonstrate_cycle(problem);
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Done!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
