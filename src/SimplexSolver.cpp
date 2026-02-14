#include "SimplexSolver.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <string>

#include "FormConverter.hpp"
#include "lib.hpp"

SimplexSolver::Solution SimplexSolver::create_infeasible_solution(int n) {
    Solution sol;
    sol.x.resize(n, 0.0);
    sol.objective_value = std::numeric_limits<double>::infinity();
    sol.basis.clear();
    sol.is_feasible = false;
    sol.is_optimal = false;
    sol.is_unbounded = false;
    sol.is_degenerate = false;
    sol.status_message = "No feasible solution exists";
    return sol;
}

// Проверяет базис на допустимость.
// x>=0
bool SimplexSolver::is_basis_feasible(const Eigen::VectorXd& xB, double tolerance) {
    return std::ranges::all_of(xB, [tolerance](double d) { return d >= -tolerance; });
}

// Проверяет базис на вырожденность.
bool SimplexSolver::is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance) {
    return std::ranges::any_of(xB, [tolerance](double d) { return std::abs(d) < tolerance; });
}

SimplexSolver::Solution SimplexSolver::solve(const LinearProgram& lp, bool verbose) {
    if (verbose) {
        lp.print("Simplex solver. Original problem:");
    }

    // Преобразуем задачу в каноническую форму
    LinearProgram canonical = FormConverter::to_canonical_form(lp);

    if (verbose && lp.getForm() != LinearProgram::CANONIC) {
        canonical.print("Problem in canonical form:");
    }

    const size_t m = canonical.num_constraints();
    const size_t n = canonical.num_variables();

    // Обработка тривиальных случаев
    if (m == 0) {
        return handle_no_constraints(n, canonical.objective(), verbose);
    }
    if (n == 0) {
        return handle_no_variables(m, canonical.rhs(), verbose);
    }

    // Преобразуем данные в структуры Eigen
    Eigen::VectorXd c = std_to_eigen(canonical.objective());
    Eigen::MatrixXd A = std_to_eigen(canonical.constraints());
    Eigen::VectorXd b = std_to_eigen(canonical.rhs());

    if (verbose) {
        std::cout << "Constraints (m): " << m << ", Original variables (n): " << n << std::endl;
        std::cout << "b = " << b.transpose() << std::endl;
    }

    // PHASE I: Поиск допустимого базиса с помощью искусственных переменных
    if (verbose) {
        std::cout << " PHASE I: Artificial Basis Method" << std::endl;
    }

    const size_t n_phase1 = n + m;
    Eigen::MatrixXd A_phase1(m, n_phase1);
    A_phase1.leftCols(n) = A;
    A_phase1.rightCols(m) = Eigen::MatrixXd::Identity(m, m);

    std::vector<double> c_phase1(n_phase1, 0.0);
    std::fill(c_phase1.begin() + n, c_phase1.end(), 1.0); // Минимизируем сумму искусственных

    // Начальный базис = искусственные переменные
    // (0,..., 0, n, n+1, ..., m-n-1)
    std::vector<size_t> basis(m);
    std::iota(basis.begin(), basis.end(), n);

    // Начальная обратная базисная матрица = единичная (базисные столбцы = I)
    Eigen::MatrixXd B_inv = Eigen::MatrixXd::Identity(m, m);
    Eigen::VectorXd x_B = b;

    auto phase1_result = run_simplex_phase(A_phase1, b, c_phase1, std::move(basis), B_inv, x_B, n, "Phase I", verbose);

    int phase1_iterations = phase1_result ? phase1_result->iterations : 0;

    if (!phase1_result || phase1_result->objective_value > EPS) {
        Solution sol = create_infeasible_solution(n);
        sol.status_message = "Problem is infeasible (Phase I objective > 0)";
        if (verbose) {
            std::cout << " PHASE I RESULT: INFEASIBLE" << std::endl;
            std::cout << "Phase I objective value: " << (phase1_result ? phase1_result->objective_value : -1.0)
                      << " > tolerance (" << EPS << ")" << std::endl;
            print_final_summary(sol, phase1_iterations, 0, verbose);
        }
        return sol;
    }

    B_inv = std::move(phase1_result->B_inv);
    x_B = std::move(phase1_result->x_B);
    basis = std::move(phase1_result->basis);

    if (verbose) {
        std::cout << " PHASE I COMPLETE" << std::endl;
        std::cout << "Feasible basis found. Phase I objective: " << phase1_result->objective_value << std::endl;
        std::cout << "Basis indices: ";
        print_vector(basis);
    }

    // Удаление искусственных переменных из базиса
    if (verbose) {
        std::cout << " Removing Artificial Variables from Basis" << std::endl;
    }

    bool cleanup_success = cleanup_artificial_basis(A, B_inv, basis, x_B, n, verbose);
    if (!cleanup_success) {
        Solution sol = create_infeasible_solution(n);
        sol.status_message = "Could not remove artificial variables from basis (redundant constraints)";
        if (verbose) {
            std::cout << " CLEANUP FAILED" << std::endl;
            print_final_summary(sol, phase1_iterations, 0, verbose);
        }
        return sol;
    }

    if (verbose) {
        std::cout << " CLEANUP COMPLETE" << std::endl;
        std::cout << "Basis for Phase II: ";
        print_vector(basis);
    }

    // ===== PHASE II: Оптимизация исходной целевой функции =====
    if (verbose) {
        std::cout << " PHASE II: Optimization" << std::endl;
    }

    auto phase2_result =
        run_simplex_phase(A, b, canonical.objective(), std::move(basis), B_inv, x_B, n, "Phase II", verbose);

    int phase2_iterations = phase2_result ? phase2_result->iterations : 0;

    if (!phase2_result) {
        Solution sol = create_infeasible_solution(n);
        sol.status_message = "Phase II failed to converge";
        if (verbose) {
            std::cout << " PHASE II FAILED" << std::endl;
            print_final_summary(sol, phase1_iterations, phase2_iterations, verbose);
        }
        return sol;
    }

    bool is_unbounded_flag = false;
    // на всякий случай перепроверяем оптимальность
    bool is_optimal_flag = check_optimality(A, c, phase2_result->basis, phase2_result->B_inv, is_unbounded_flag);

    if (verbose && phase2_result->optimal != is_optimal_flag) {
        std::cout << "\n[WARNING] Mismatch between iteration termination and explicit optimality check:"
                  << "\n  Iteration termination: " << (phase2_result->optimal ? "optimal" : "not optimal")
                  << "\n  Explicit check: " << (is_optimal_flag ? "optimal" : "not optimal") << std::endl;
    }

    // Формируем полное решение в канонической форме
    std::vector<double> x_canonical(n, 0.0);
    for (size_t i = 0; i < m; ++i) {
        if (phase2_result->basis[i] < n) {
            x_canonical[phase2_result->basis[i]] = phase2_result->x_B[i];
        }
    }

    // Восстанавливаем решение исходной задачи
    std::vector<double> x_original = restore_original_solution(lp, x_canonical);

    // Корректируем знак целевой функции для задач максимизации
    double final_objective = phase2_result->objective_value;
    if (!lp.is_minimization()) {
        final_objective = -final_objective;
    }

    // Формируем результат
    Solution sol;
    sol.x = std::move(x_original);
    sol.objective_value = final_objective;
    sol.basis = std::move(phase2_result->basis);
    sol.is_feasible = true;
    sol.is_optimal = is_optimal_flag;
    sol.is_unbounded = phase2_result->unbounded;
    sol.is_degenerate = is_basis_degenerate(phase2_result->x_B, EPS);
    sol.status_message = phase2_result->unbounded
        ? "Unbounded problem"
        : (phase2_result->optimal ? "Optimal solution found" : "Feasible solution found");

    print_final_summary(sol, phase1_iterations, phase2_iterations, verbose);
    return sol;
}

// Единая функция симплекс-итераций
std::optional<SimplexSolver::PhaseResult>
SimplexSolver::run_simplex_phase(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const std::vector<double>& c,
                                 std::vector<size_t> basis, Eigen::MatrixXd B_inv, Eigen::VectorXd x_B,
                                 size_t n_active_vars, const std::string& phase_name, bool verbose) {

    const size_t m = A.rows();
    int iter = 0;
    bool unbounded = false;

    while (iter < MAX_ITERATIONS) {
        ++iter;

        // Шаг 1: Вычисляем двойственные переменные y = c_B^T * B^{-1}
        Eigen::VectorXd c_B(m);
        for (size_t i = 0; i < m; ++i)
            c_B[i] = c[basis[i]];
        Eigen::RowVectorXd y = c_B.transpose() * B_inv;

        // Шаг 2: Вычисляем приведённые стоимости d_j = c_j - y * A_j
        std::vector<double> d(n_active_vars, 0.0);
        size_t entering = n_active_vars;
        double min_d = 0.0;
        bool optimal = true;

        for (size_t j = 0; j < n_active_vars; ++j) {
            if (std::ranges::find(basis, j) != basis.end()) {
                continue;
            }

            d[j] = c[j] - y * A.col(j);

            if (d[j] < -EPS) {
                optimal = false;
                if (entering == n_active_vars || d[j] < min_d) {
                    min_d = d[j];
                    entering = j;
                }
            }
        }

        if (optimal) {
            if (verbose) {
                std::cout << "\n[" << phase_name << "] Optimal solution reached at iteration " << iter << std::endl;
            }
            break;
        }

        if (entering == n_active_vars) {
            if (verbose) {
                std::cerr << "\n[" << phase_name
                          << "] Warning: No entering variable found despite non-optimal reduced costs" << std::endl;
            }
            break;
        }

        // Шаг 3: Вычисляем направление u = B^{-1} * A_entering
        Eigen::VectorXd u = B_inv * A.col(entering);

        // Шаг 4: Проверка на неограниченность (все u_i <= 0)
        bool has_positive = false;
        for (size_t i = 0; i < m; ++i) {
            if (u[i] > EPS) {
                has_positive = true;
                break;
            }
        }
        if (!has_positive) {
            unbounded = true;
            if (verbose) {
                std::cout << "\n[" << phase_name << "] Problem is unbounded at iteration " << iter << std::endl;
            }
            break;
        }

        // Шаг 5: Вычисляем длину шага theta = min{x_i/u_i | u_i > 0}
        double theta = std::numeric_limits<double>::max();
        size_t leaving_pos = m;
        for (size_t i = 0; i < m; ++i) {
            if (u[i] > EPS) {
                double ratio = x_B[i] / u[i];
                if (ratio < theta - EPS) {
                    theta = ratio;
                    leaving_pos = i;
                }
            }
        }

        if (leaving_pos == m) {
            if (verbose) {
                std::cerr << "\n[" << phase_name << "] Error: No leaving variable found" << std::endl;
            }
            return std::nullopt;
        }

        // Логируем смену базиса
        size_t leaving_var = basis[leaving_pos];
        bool degenerate = (std::abs(theta) < EPS);

        // Обновляем базисное решение
        x_B = x_B - theta * u;
        x_B[leaving_pos] = theta;

        // Обновляем базис
        basis[leaving_pos] = entering;

        // Обновляем B^{-1} через матрицу F
        const double pivot = u[leaving_pos];
        const Eigen::RowVectorXd row_l = B_inv.row(leaving_pos);
        for (size_t i = 0; i < m; ++i) {
            if (i == leaving_pos) {
                B_inv.row(i) = row_l / pivot;
            } else {
                B_inv.row(i) = B_inv.row(i) - (u[i] / pivot) * row_l;
            }
        }

        // Вычисляем текущее значение целевой функции
        double obj_value = 0.0;
        for (size_t i = 0; i < m; ++i)
            obj_value += c[basis[i]] * x_B[i];

        print_basis_change(phase_name, iter, entering, leaving_var, leaving_pos, min_d, theta, degenerate, basis, x_B,
                           obj_value, verbose);
    }

    // Вычисляем финальное значение целевой функции
    double obj_value = 0.0;
    for (size_t i = 0; i < m; ++i)
        obj_value += c[basis[i]] * x_B[i];

    return PhaseResult{iter < MAX_ITERATIONS && !unbounded, // optimal flag
                       unbounded,
                       iter,
                       std::move(B_inv),
                       std::move(x_B),
                       std::move(basis),
                       obj_value};
}

// Удаление искусственных переменных из базиса
bool SimplexSolver::cleanup_artificial_basis(const Eigen::MatrixXd& A_orig, Eigen::MatrixXd& B_inv,
                                             std::vector<size_t>& basis, Eigen::VectorXd& x_B, size_t n_orig,
                                             bool verbose) {

    const size_t m = A_orig.rows();
    bool basis_changed;
    int cleanup_iter = 0;
    constexpr int MAX_CLEANUP = 100;

    do {
        basis_changed = false;
        ++cleanup_iter;

        for (size_t k = 0; k < m; ++k) {
            if (basis[k] >= n_orig) {
                // Искусственная переменная в базисе
                // Ищем небазисную оригинальную переменную с ненулевым коэффициентом в строке k
                size_t entering = n_orig;
                for (size_t j = 0; j < n_orig; ++j) {
                    if (std::ranges::find(basis, j) != basis.end())
                        continue;

                    // Вычисляем коэффициент: (B^{-1} * A_j)[k]
                    double coeff = 0.0;
                    for (size_t i = 0; i < m; ++i) {
                        coeff += B_inv(k, i) * A_orig(i, j);
                    }
                    if (std::abs(coeff) > EPS) {
                        entering = j;
                        break;
                    }
                }

                if (entering < n_orig) {
                    // Выполняем вырожденную замену (theta = 0)
                    Eigen::VectorXd u = B_inv * A_orig.col(entering);
                    const double pivot = u[k];
                    const Eigen::RowVectorXd row_l = B_inv.row(k);

                    // Обновляем базис
                    size_t leaving_var = basis[k];
                    basis[k] = entering;
                    basis_changed = true;

                    // Обновляем B^{-1} через матрицу F
                    for (size_t i = 0; i < m; ++i) {
                        if (i == k) {
                            B_inv.row(i) = row_l / pivot;
                        } else {
                            B_inv.row(i) = B_inv.row(i) - (u[i] / pivot) * row_l;
                        }
                    }

                    if (verbose) {
                        std::cout << "[Cleanup Iter " << cleanup_iter << "] Removed artificial x_" << leaving_var
                                  << " (pos " << k << "), added x_" << entering << std::endl;
                    }
                    break; // Перезапускаем проверку после изменения базиса
                }

                // невозможно удалить искусственную переменную
                if (verbose) {
                    std::cerr << "Warning: Redundant constraint at row " << k
                              << " (artificial variable cannot be removed)" << std::endl;
                }
                return false;
            }
        }
    } while (basis_changed && cleanup_iter < MAX_CLEANUP);

    // Проверяем, что все искусственные переменные удалены
    for (size_t k = 0; k < m; ++k) {
        if (basis[k] >= n_orig)
            return false;
    }
    return true;
}

bool SimplexSolver::check_optimality(const Eigen::MatrixXd& A, const Eigen::VectorXd& c,
                                     const std::vector<size_t>& basis, const Eigen::MatrixXd& B_inv,
                                     bool& is_unbounded) {
    is_unbounded = false;
    const size_t m = basis.size();
    const size_t n = c.size();

    // Шаг 1: Вычисляем двойственные переменные y = c_B^T * B^{-1} (формула 5.2)
    Eigen::VectorXd c_B(m);
    for (size_t i = 0; i < m; ++i) {
        c_B[i] = c[basis[i]];
    }
    Eigen::RowVectorXd y = c_B.transpose() * B_inv;

    // Шаг 2: Проверяем приведённые стоимости для всех небазисных переменных (формулы 5.4-5.5)
    for (size_t j = 0; j < n; ++j) {
        // Пропускаем базисные переменные
        if (std::ranges::find(basis, j) != basis.end())
            continue;

        // Приведённая стоимость: d_j = c_j - y * A_j (формула 5.3)
        double reduced_cost = c(j) - y.dot(A.col(j));

        // Для задачи минимизации оптимальность достигается при d_j >= 0 (после формулы 5.5)
        if (reduced_cost < -EPS) {
            // Проверяем на неограниченность: можно ли увеличивать x_j бесконечно?
            // Направление: u = B^{-1} * A_j (формула 5.9)
            Eigen::VectorXd direction = B_inv * A.col(j);

            // Если все компоненты направления <= 0, то можем увеличивать x_j до бесконечности
            bool can_increase_indefinitely = true;
            for (size_t i = 0; i < m; ++i) {
                if (direction[i] > EPS) {
                    can_increase_indefinitely = false;
                    break;
                }
            }

            if (can_increase_indefinitely) {
                is_unbounded = true;
                return false; // Неограниченная задача
            }

            // Иначе решение не оптимально (есть улучшающее направление)
            return false;
        }
    }

    return true; // Все приведённые стоимости неотрицательны -> оптимальное решение
}


// Вывод информации о смене базиса
void SimplexSolver::print_basis_change(const std::string& phase_name, int iteration, size_t entering_var,
                                       size_t leaving_var, size_t leaving_pos, double reduced_cost, double theta,
                                       bool degenerate, const std::vector<size_t>& new_basis,
                                       const Eigen::VectorXd& new_xB, double objective_value, bool verbose) {

    if (!verbose) {
        return;
    }

    // Выводим первые 10 итераций и все итерации с вырожденными замены
    if (iteration <= 10 || degenerate) {
        std::cout << "\n[" << phase_name << " Iter " << iteration << (degenerate ? " (DEGENERATE)" : "")
                  << "] Basis change:" << std::endl;

        std::cout << "  Entering variable: x_" << entering_var << " (reduced cost = " << reduced_cost << ")"
                  << std::endl;
        std::cout << "  Leaving variable:  x_" << leaving_var << " (position " << leaving_pos << " in basis)"
                  << std::endl;
        std::cout << "  Step length theta: " << theta << (degenerate ? " (degenerate pivot)" : "") << std::endl;

        std::cout << "  New basis: ";
        print_vector(new_basis);

        std::cout << "  Basic variables: (";
        std::cout << new_xB << std::endl;
        std::cout << ")" << std::endl;

        std::cout << "  Objective value: " << objective_value << std::endl;
    } else if (iteration == 11) {
        std::cout << "\n  ... (showing only first 10 iterations in detail) ..." << std::endl;
    }
}

// Вывод итоговой сводки
void SimplexSolver::print_final_summary(const Solution& sol, int phase1_iterations, int phase2_iterations,
                                        bool verbose) {
    if (!verbose) {
        return;
    }
    std::cout << "SIMPLEX METHOD COMPLETE: FINAL SUMMARY" << std::endl;

    std::cout << "\nIteration statistics:" << std::endl;
    std::cout << "  Phase I iterations:   " << phase1_iterations << std::endl;
    std::cout << "  Phase II iterations:  " << phase2_iterations << std::endl;

    if (!sol.is_feasible) {
        std::cout << "\n>>> RESULT: INFEASIBLE PROBLEM <<<" << std::endl;
        std::cout << "  Status: " << sol.status_message << std::endl;
        return;
    }

    std::cout << "\nOptimal solution found:" << std::endl;
    std::cout << "  Objective value: " << sol.objective_value << std::endl;

    std::cout << "  Solution vector (original variables): ";
    print_vector(sol.x);


    std::cout << "  Basis indices in canonical form: ";
    print_vector(sol.basis);

    if (sol.is_unbounded) {
        std::cout << "\n  *** WARNING: PROBLEM IS UNBOUNDED ***" << std::endl;
        std::cout << "      Objective can be improved indefinitely while maintaining feasibility." << std::endl;
    } else if (sol.is_optimal) {
        std::cout << "\n  *** SOLUTION IS OPTIMAL ***" << std::endl;
        std::cout << "      All reduced costs are non-negative (minimization problem)." << std::endl;
    }

    if (sol.is_degenerate) {
        std::cout << "\n  *** NOTE: SOLUTION IS DEGENERATE ***" << std::endl;
        std::cout << "      At least one basic variable is approximately zero." << std::endl;
    }

    std::cout << "\nStatus: " << sol.status_message << std::endl;
}

// Обработка тривиальных случаев
SimplexSolver::Solution SimplexSolver::handle_no_constraints(size_t n, const std::vector<double>& c, bool verbose) {
    if (verbose) {
        std::cout << " TRIVIAL CASE: No constraints" << std::endl;
        std::cout << "Variables: " << n << std::endl;
    }

    bool unbounded = std::ranges::any_of(c, [](double v) { return v < -EPS; });

    Solution sol;
    sol.x.resize(n, 0.0);
    sol.objective_value = unbounded ? -std::numeric_limits<double>::infinity() : 0.0;
    sol.basis.clear();
    sol.is_feasible = true;
    sol.is_optimal = !unbounded;
    sol.is_unbounded = unbounded;
    sol.is_degenerate = false;
    sol.status_message =
        unbounded ? "Unbounded problem (no constraints, negative cost coefficient)" : "Optimal solution at x=0";

    if (verbose) {
        std::cout << "Result: " << (unbounded ? "UNBOUNDED" : "OPTIMAL at x=0") << std::endl;
        std::cout << "Objective value: " << sol.objective_value << std::endl;
    }

    return sol;
}

SimplexSolver::Solution SimplexSolver::handle_no_variables(size_t m, const std::vector<double>& b, bool verbose) {
    if (verbose) {
        std::cout << " TRIVIAL CASE: No variables" << std::endl;
        std::cout << "Constraints: " << m << std::endl;
    }

    bool feasible = std::ranges::all_of(b, [](double v) { return std::abs(v) < EPS; });

    Solution sol = create_infeasible_solution(0);
    if (feasible) {
        sol.is_feasible = true;
        sol.objective_value = 0.0;
        sol.status_message = "Feasible (0=0 constraints)";
    } else {
        sol.status_message = "Infeasible (0=b, b!=0)";
    }

    if (verbose) {
        std::cout << "Result: " << (feasible ? "FEASIBLE" : "INFEASIBLE") << std::endl;
        std::cout << "Status: " << sol.status_message << std::endl;
    }

    return sol;
}
