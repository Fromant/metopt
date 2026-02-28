#pragma once

#include <cassert>
#include <random>
#include <vector>

#include "linear/DualBuilder.hpp"
#include "linear/LinearProgram.hpp"
#include "linear/solvers/SimplexSolver.hpp"
#include "transport/TransportProblem.hpp"
#include "transport/TransportSolver.hpp"


inline std::vector<double> restore_original_solution(const LinearProgram& original_lp,
                                                     const std::vector<double>& canonical_solution) {
    const int n_orig = original_lp.num_variables();
    std::vector<double> original_solution(n_orig, 0.0);

    // Определяем позиции "минус" частей для свободных переменных
    // Пример: если свободные переменные имеют индексы [3, 4] в исходной задаче,
    // то их "минус" части будут находиться по индексам [n_orig + 0, n_orig + 1]
    std::vector<int> free_neg_indices;
    for (int i = 0; i < n_orig; ++i) {
        if (original_lp.var_constraints()[i] == "free") {
            free_neg_indices.push_back(n_orig + static_cast<int>(free_neg_indices.size()));
        }
    }

    // Восстанавливаем значения переменных
    int free_counter = 0;
    for (int i = 0; i < n_orig; ++i) {
        if (original_lp.var_constraints()[i] == "free") {
            // x_free = x+ - x-
            double pos_part = (i < static_cast<int>(canonical_solution.size())) ? canonical_solution[i] : 0.0;
            double neg_part = 0.0;

            if (free_counter < static_cast<int>(free_neg_indices.size()) &&
                free_neg_indices[free_counter] < static_cast<int>(canonical_solution.size())) {
                neg_part = canonical_solution[free_neg_indices[free_counter]];
            }

            original_solution[i] = pos_part - neg_part;
            free_counter++;
        } else {
            original_solution[i] = (i < static_cast<int>(canonical_solution.size())) ? canonical_solution[i] : 0.0;
        }
    }

    return original_solution;
}

template <typename T>
void print_vector(const std::vector<T>& v) {
    std::cout << "(";
    if (!v.empty()) {
        for (int i = 0; i < v.size() - 1; i++) {
            std::cout << v[i] << ", ";
        }
        std::cout << v[v.size() - 1];
    }
    std::cout << ")" << std::endl;
}

inline Eigen::VectorXd std_to_eigen(const std::vector<double>& v) {
    Eigen::VectorXd result;
    result.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i];
    }
    return result;
}

inline Eigen::VectorXd std_to_eigen(const std::vector<double>& v, const std::function<double(double)>& transform) {
    Eigen::VectorXd result;
    result.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = transform(v[i]);
    }
    return result;
}

inline Eigen::MatrixXd std_to_eigen(const std::vector<std::vector<double>>& m,
                                    const std::function<double(double)>& transform) {
    Eigen::MatrixXd result;
    result.resize(m.size(), m[0].size());
    for (size_t i = 0; i < m.size(); ++i) {
        for (size_t j = 0; j < m[i].size(); ++j) {
            result(i, j) = transform(m[i][j]);
        }
    }
    return result;
}

inline Eigen::MatrixXd std_to_eigen(const std::vector<std::vector<double>>& m) {
    Eigen::MatrixXd result;
    result.resize(m.size(), m[0].size());
    for (size_t i = 0; i < m.size(); ++i) {
        for (size_t j = 0; j < m[i].size(); ++j) {
            result(i, j) = m[i][j];
        }
    }
    return result;
}

inline auto get_val_with_err(const double val1, const double val2) {
    double err = std::abs(val1 - val2) / 2;
    double avg = (val1 + val2) / 2;

    const double exponent = std::floor(std::log10(err));
    // Разряд последней значащей цифры погрешности: -exponent
    const auto precision = std::clamp(-exponent, 4.0, 10.0);
    const double factor = std::pow(10.0, -precision - 1); // Сохраняем 2 значащие цифры
    err = std::round(err / factor) * factor;
    avg = std::round(avg / factor) * factor;

    return std::make_tuple(avg, err, precision);
}

inline void print_val_with_err(const double val1, const double val2) {
    double err = std::abs(val1 - val2) / 2;
    double avg = (val1 + val2) / 2;

    const double exponent = std::floor(std::log10(err));
    // Разряд последней значащей цифры погрешности: -exponent
    const auto precision = std::clamp(-exponent, 4.0, 10.0);
    const double factor = std::pow(10.0, -precision - 1); // Сохраняем 2 значащие цифры
    err = std::round(err / factor) * factor;
    avg = std::round(avg / factor) * factor;
    std::cout << std::fixed << std::setprecision(precision) << avg << " +- " << err << std::endl;
    const double rel_err = (err / std::abs(avg)) * 100.0;
    std::cout << "Relative error: " << std::fixed << std::setprecision(2) << rel_err << " %" << std::endl;
}

inline std::optional<std::tuple<double, double, double>>
solve_lp_simplex(const LinearProgram& lp, bool printSolution = false, bool verbose = false) {
    const auto directSolution = SimplexSolver::solve(lp, verbose);

    const auto dualProblem = DualBuilder::build_dual(lp);

    const auto dualSolution = SimplexSolver::solve(dualProblem, verbose);

    if (!directSolution.is_feasible || !dualSolution.is_feasible || directSolution.is_unbounded ||
        dualSolution.is_unbounded) {
        return std::nullopt;
    }

    return get_val_with_err(directSolution.objective_value, dualSolution.objective_value);
}

inline TransportSolution solve_transport_with_modi(const TransportProblem& problem, bool printSolution = false,
                                                   bool verbose = false) {
    if (verbose) {
        std::cout << "\n========== MODI METHOD ==========" << std::endl;
    }

    if (verbose && !problem.isBalanced()) {
        std::cout << "Problem is not balanced. Balancing..." << std::endl;
    }

    TransportProblem balanced = problem.balance();
    if (printSolution) {
        balanced.print("Balanced Problem");
    }

    auto result = TransportSolver::solve(problem, verbose);

    return result;
}

inline std::optional<TransportSolution> solve_transport_with_simplex(const TransportProblem& original_problem,
                                                                     bool printSolution = false, bool verbose = false) {
    if (verbose) {
        printSolution = true;
    }
    if (printSolution) {
        std::cout << "\n========== SIMPLEX METHOD ==========" << std::endl;
    }

    TransportProblem expanded = TransportProblem::createExpandedProblem(original_problem);
    LinearProgram lp = expanded.toLinearProgram();

    if (verbose) {
        lp.print("LP form of transport problem");
    }

    auto result = SimplexSolver::solve(lp, false);

    if (!result.is_feasible) {
        if (printSolution) {
            std::cout << "ERROR: Problem is infeasible" << std::endl;
        }
        return std::nullopt;
    }

    if (result.is_unbounded) {
        if (printSolution) {
            std::cout << "ERROR: Problem is unbounded" << std::endl;
        }
        return std::nullopt;
    }

    if (!result.is_optimal) {
        if (printSolution) {
            std::cout << "WARNING: Solution is not optimal" << std::endl;
        }
    }

    auto plan = TransportProblem::restorePlanFromVector(result.x, original_problem.numSuppliers(),
                                                        original_problem.numConsumers(), expanded.numSuppliers(),
                                                        expanded.numConsumers());

    if (printSolution) {
        std::cout << "\n=== Simplex Solution ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Objective value: " << result.objective_value << "\n";
        std::cout << "Basis size: " << result.basis.size() << "\n";

        TransportProblem::printPlan(plan, original_problem.supplies(), original_problem.demands(),
                                    "Optimal Transportation Plan (Simplex)");
    }

    return TransportSolution{plan, {}, result.objective_value, result.objective_value, 0, true, 0, {}};
}

inline std::optional<double> solve_transport_problem(const TransportProblem& original_problem,
                                                     bool printSolution = false, bool verbose = false) {
    const auto sol1 = solve_transport_with_modi(original_problem, printSolution, verbose);
    const auto sol2_opt = solve_transport_with_simplex(original_problem, printSolution, verbose);
    if (!sol2_opt) {
        return std::nullopt;
    }
    const auto sol2 = *sol2_opt;

    const auto [avg, err, precision] = get_val_with_err(sol1.total_cost, sol2.total_cost);
    if (printSolution) {
        std::cout << "\n=== Comparison ===" << std::endl;
        std::cout << "NWA + potentials cost:    " << sol1.total_cost << "\n";
        std::cout << "Simplex cost: " << sol2.total_cost << "\n";
        std::cout << "Final average cost: " << std::fixed << std::setprecision(precision) << avg << " +- " << err
                  << std::endl;
        std::cout << "Relative error: " << std::fixed << std::setprecision(2) << err / avg << std::endl;
    }
    return avg;
}
