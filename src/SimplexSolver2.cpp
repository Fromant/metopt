#include "SimplexSolver2.hpp"
#include "FormConverter.hpp"
#include "LinearProgram.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <string>

// ============================================================================
// Вспомогательные функции
// ============================================================================

SimplexSolver2::Solution SimplexSolver2::create_infeasible_solution(int n) {
    Solution sol;
    sol.x.resize(n, 0.0);
    sol.objective_value = std::numeric_limits<double>::infinity();
    sol.basis.clear();
    sol.is_feasible = false;
    sol.is_optimal = false;
    sol.is_unbounded = false;
    sol.is_degenerate = false;
    sol.status_message = "No feasible solution exists (empty feasible region)";
    return sol;
}

bool SimplexSolver2::is_basis_feasible(const Eigen::VectorXd& xB, double tolerance) {
    return !std::ranges::any_of(xB, [tolerance](double d) { return d < -tolerance; });
}

bool SimplexSolver2::is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance) {
    return std::ranges::any_of(xB, [tolerance](double d) { return std::abs(d) < tolerance; });
}

// ============================================================================
// Восстановление решения исходной задачи из канонической формы
// ============================================================================

std::vector<double> SimplexSolver2::restore_original_solution(const LinearProgram& original_lp,
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
            // Неотрицательные переменные берём напрямую
            original_solution[i] = (i < static_cast<int>(canonical_solution.size())) ? canonical_solution[i] : 0.0;
        }
    }

    return original_solution;
}

// ============================================================================
// Основной метод решения
// ============================================================================

SimplexSolver2::Solution SimplexSolver2::solve(const LinearProgram& lp, bool verbose) {
    if (verbose) {
        lp.print("Simplex solver. Original problem:");
    }

    // Преобразуем задачу в каноническую форму
    LinearProgram canonical = FormConverter::to_canonical_form(lp);

    if (verbose && lp.getForm() != LinearProgram::CANONIC) {
        canonical.print("Problem in canonical form:");
    }

    const size_t m = canonical.num_constraints();
    const size_t n_orig = canonical.num_variables();

    // Обработка тривиальных случаев
    if (m == 0) return handle_no_constraints(n_orig, canonical.objective(), verbose);
    if (n_orig == 0) return handle_no_variables(m, canonical.rhs(), verbose);

    // Преобразуем данные в структуры Eigen
    Eigen::VectorXd b(m);
    for (size_t i = 0; i < m; ++i) b(i) = canonical.rhs()[i];

    Eigen::MatrixXd A(m, n_orig);
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j < n_orig; ++j)
            A(i, j) = canonical.constraints()[i][j];

    Eigen::VectorXd c_orig(n_orig);
    for (size_t i = 0; i < n_orig; ++i) c_orig(i) = canonical.objective()[i];

    if (verbose) {
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << " SIMPLEX METHOD START" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "Constraints (m): " << m << ", Original variables (n): " << n_orig << std::endl;
        std::cout << "b = " << b.transpose() << std::endl;
    }

    // ===== PHASE I: Поиск допустимого базиса с помощью искусственных переменных =====
    if (verbose) {
        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << " PHASE I: Artificial Basis Method" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
    }

    const size_t n_phase1 = n_orig + m;
    Eigen::MatrixXd A_phase1(m, n_phase1);
    A_phase1.leftCols(n_orig) = A;
    A_phase1.rightCols(m) = Eigen::MatrixXd::Identity(m, m);

    std::vector<double> c_phase1(n_phase1, 0.0);
    std::fill(c_phase1.begin() + n_orig, c_phase1.end(), 1.0); // Минимизируем сумму искусственных

    // Начальный базис = искусственные переменные
    std::vector<size_t> basis(m);
    std::iota(basis.begin(), basis.end(), n_orig);

    // Начальная обратная базисная матрица = единичная (базисные столбцы = I)
    Eigen::MatrixXd B_inv = Eigen::MatrixXd::Identity(m, m);
    Eigen::VectorXd x_B = b;

    auto phase1_result = run_simplex_phase(
        A_phase1, b, c_phase1, std::move(basis), B_inv, x_B,
        n_orig,  // Только оригинальные переменные могут войти в базис при очистке
        "Phase I", verbose
    );

    int phase1_iterations = phase1_result ? phase1_result->iterations : 0;

    if (!phase1_result || phase1_result->objective_value > TOLERANCE) {
        Solution sol = create_infeasible_solution(n_orig);
        sol.status_message = "Problem is infeasible (Phase I objective > 0)";
        if (verbose) {
            std::cout << "\n" << std::string(70, '=') << std::endl;
            std::cout << " PHASE I RESULT: INFEASIBLE" << std::endl;
            std::cout << std::string(70, '=') << std::endl;
            std::cout << "Phase I objective value: " << (phase1_result ? phase1_result->objective_value : -1.0)
                      << " > tolerance (" << TOLERANCE << ")" << std::endl;
            print_final_summary(sol, phase1_iterations, 0, 0, verbose);
        }
        return sol;
    }

    B_inv = std::move(phase1_result->B_inv);
    x_B = std::move(phase1_result->x_B);
    basis = std::move(phase1_result->basis);

    if (verbose) {
        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << " PHASE I COMPLETE" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
        std::cout << "Feasible basis found. Phase I objective: " << phase1_result->objective_value << std::endl;
        std::cout << "Basis indices: {";
        for (size_t i = 0; i < basis.size(); ++i) {
            std::cout << basis[i];
            if (i < basis.size() - 1) std::cout << ", ";
        }
        std::cout << "}" << std::endl;
    }

    // ===== ОЧИСТКА: Удаление искусственных переменных из базиса =====
    if (verbose) {
        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << " CLEANUP: Removing Artificial Variables from Basis" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
    }

    int cleanup_iterations = 0;
    bool cleanup_success = cleanup_artificial_basis(A, B_inv, basis, x_B, n_orig, verbose);
    if (!cleanup_success) {
        Solution sol = create_infeasible_solution(n_orig);
        sol.status_message = "Could not remove artificial variables from basis (redundant constraints)";
        if (verbose) {
            std::cout << "\n" << std::string(70, '=') << std::endl;
            std::cout << " CLEANUP FAILED" << std::endl;
            std::cout << std::string(70, '=') << std::endl;
            print_final_summary(sol, phase1_iterations, cleanup_iterations, 0, verbose);
        }
        return sol;
    }

    if (verbose) {
        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << " CLEANUP COMPLETE" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
        std::cout << "Basis for Phase II: {";
        for (size_t i = 0; i < basis.size(); ++i) {
            std::cout << basis[i];
            if (i < basis.size() - 1) std::cout << ", ";
        }
        std::cout << "}" << std::endl;
    }

    // ===== PHASE II: Оптимизация исходной целевой функции =====
    if (verbose) {
        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << " PHASE II: Optimization" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
    }

    auto phase2_result = run_simplex_phase(
        A, b, canonical.objective(), std::move(basis), B_inv, x_B,
        n_orig,  // Все оригинальные переменные участвуют
        "Phase II", verbose
    );

    int phase2_iterations = phase2_result ? phase2_result->iterations : 0;

    if (!phase2_result) {
        Solution sol = create_infeasible_solution(n_orig);
        sol.status_message = "Phase II failed to converge";
        if (verbose) {
            std::cout << "\n" << std::string(70, '=') << std::endl;
            std::cout << " PHASE II FAILED" << std::endl;
            std::cout << std::string(70, '=') << std::endl;
            print_final_summary(sol, phase1_iterations, cleanup_iterations, phase2_iterations, verbose);
        }
        return sol;
    }

    // Формируем полное решение в канонической форме
    std::vector<double> x_canonical(n_orig, 0.0);
    for (size_t i = 0; i < m; ++i) {
        if (phase2_result->basis[i] < n_orig) {
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
    sol.is_optimal = phase2_result->optimal;
    sol.is_unbounded = phase2_result->unbounded;
    sol.is_degenerate = is_basis_degenerate(phase2_result->x_B, TOLERANCE);
    sol.status_message = phase2_result->unbounded
                             ? "Unbounded problem (objective can be improved indefinitely)"
                             : (phase2_result->optimal ? "Optimal solution found"
                                                       : "Feasible solution found (may not be optimal)");

    print_final_summary(sol, phase1_iterations, cleanup_iterations, phase2_iterations, verbose);
    return sol;
}

// ============================================================================
// Единый движок симплекс-метода для обеих фаз
// ============================================================================

std::optional<SimplexSolver2::PhaseResult> SimplexSolver2::run_simplex_phase(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<double>& c,
    std::vector<size_t> basis,
    Eigen::MatrixXd B_inv,
    Eigen::VectorXd x_B,
    size_t n_active_vars,
    const std::string& phase_name,
    bool verbose) {

    const size_t m = A.rows();
    int iter = 0;
    bool unbounded = false;

    while (iter < MAX_ITERATIONS) {
        ++iter;

        // Шаг 1: Вычисляем двойственные переменные y = c_B^T * B^{-1}
        Eigen::VectorXd c_B(m);
        for (size_t i = 0; i < m; ++i) c_B[i] = c[basis[i]];
        Eigen::RowVectorXd y = c_B.transpose() * B_inv;

        // Шаг 2: Вычисляем приведённые стоимости d_j = c_j - y * A_j
        std::vector<double> d(n_active_vars, 0.0);
        size_t entering = n_active_vars;
        double min_d = 0.0;
        bool optimal = true;

        for (size_t j = 0; j < n_active_vars; ++j) {
            if (std::find(basis.begin(), basis.end(), j) != basis.end()) continue;

            double dj = c[j];
            for (size_t i = 0; i < m; ++i) dj -= y[i] * A(i, j);
            d[j] = dj;

            if (dj < -TOLERANCE) {
                optimal = false;
                if (entering == n_active_vars || dj < min_d) {
                    min_d = dj;
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
                std::cerr << "\n[" << phase_name << "] Warning: No entering variable found despite non-optimal reduced costs"
                          << std::endl;
            }
            break;
        }

        // Шаг 3: Вычисляем направление u = B^{-1} * A_entering
        Eigen::VectorXd u = B_inv * A.col(entering);

        // Шаг 4: Проверка на неограниченность (все u_i <= 0)
        bool has_positive = false;
        for (size_t i = 0; i < m; ++i) {
            if (u[i] > TOLERANCE) {
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
            if (u[i] > TOLERANCE) {
                double ratio = x_B[i] / u[i];
                if (ratio < theta - TOLERANCE) {
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
        bool degenerate = (std::abs(theta) < TOLERANCE);

        // Обновляем базисное решение
        x_B = x_B - theta * u;
        x_B[leaving_pos] = theta;

        // Обновляем базис
        basis[leaving_pos] = entering;

        // Обновляем B^{-1} через матрицу Ф (формулы 5.8-5.10 из методички)
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
        for (size_t i = 0; i < m; ++i) obj_value += c[basis[i]] * x_B[i];

        print_basis_change(phase_name, iter, entering, leaving_var, leaving_pos, min_d, theta, degenerate,
                           basis, x_B, obj_value, verbose);
    }

    // Вычисляем финальное значение целевой функции
    double obj_value = 0.0;
    for (size_t i = 0; i < m; ++i) obj_value += c[basis[i]] * x_B[i];

    return PhaseResult{
        iter < MAX_ITERATIONS && !unbounded,  // optimal flag
        unbounded,
        iter,
        std::move(B_inv),
        std::move(x_B),
        std::move(basis),
        obj_value
    };
}

// ============================================================================
// Удаление искусственных переменных из базиса
// ============================================================================

bool SimplexSolver2::cleanup_artificial_basis(
    const Eigen::MatrixXd& A_orig,
    Eigen::MatrixXd& B_inv,
    std::vector<size_t>& basis,
    Eigen::VectorXd& x_B,
    size_t n_orig,
    bool verbose) {

    const size_t m = A_orig.rows();
    bool basis_changed;
    int cleanup_iter = 0;
    constexpr int MAX_CLEANUP = 100;

    do {
        basis_changed = false;
        ++cleanup_iter;

        for (size_t k = 0; k < m; ++k) {
            if (basis[k] >= n_orig) {  // Искусственная переменная в базисе
                // Ищем небазисную оригинальную переменную с ненулевым коэффициентом в строке k
                size_t entering = n_orig;
                for (size_t j = 0; j < n_orig; ++j) {
                    if (std::find(basis.begin(), basis.end(), j) != basis.end()) continue;

                    // Вычисляем коэффициент: (B^{-1} * A_j)[k]
                    double coeff = 0.0;
                    for (size_t i = 0; i < m; ++i) {
                        coeff += B_inv(k, i) * A_orig(i, j);
                    }
                    if (std::abs(coeff) > TOLERANCE) {
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

                    // Обновляем B^{-1} через матрицу Ф
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
                    break;  // Перезапускаем проверку после изменения базиса
                } else {
                    // Избыточное ограничение - невозможно удалить искусственную переменную
                    if (verbose) {
                        std::cerr << "Warning: Redundant constraint at row " << k
                                  << " (artificial variable cannot be removed)" << std::endl;
                    }
                    return false;
                }
            }
        }
    } while (basis_changed && cleanup_iter < MAX_CLEANUP);

    // Проверяем, что все искусственные переменные удалены
    for (size_t k = 0; k < m; ++k) {
        if (basis[k] >= n_orig) return false;
    }
    return true;
}

// ============================================================================
// Вывод информации о смене базиса
// ============================================================================

void SimplexSolver2::print_basis_change(
    const std::string& phase_name,
    int iteration,
    size_t entering_var,
    size_t leaving_var,
    size_t leaving_pos,
    double reduced_cost,
    double theta,
    bool degenerate,
    const std::vector<size_t>& new_basis,
    const Eigen::VectorXd& new_xB,
    double objective_value,
    bool verbose) {

    if (!verbose) return;

    // Показываем детали для первых 10 итераций или при вырожденных заменах
    if (iteration <= 10 || degenerate) {
        std::cout << "\n[" << phase_name << " Iter " << iteration
                  << (degenerate ? " (DEGENERATE)" : "") << "] Basis change:" << std::endl;

        std::cout << "  Entering variable: x_" << entering_var
                  << " (reduced cost = " << std::fixed << std::setprecision(6) << reduced_cost << ")" << std::endl;
        std::cout << "  Leaving variable:  x_" << leaving_var << " (position " << leaving_pos << " in basis)" << std::endl;
        std::cout << "  Step length theta: " << theta << (degenerate ? " (degenerate pivot)" : "") << std::endl;

        std::cout << "  New basis: {";
        for (size_t i = 0; i < new_basis.size(); ++i) {
            std::cout << new_basis[i];
            if (i < new_basis.size() - 1) std::cout << ", ";
        }
        std::cout << "}" << std::endl;

        std::cout << "  Basic variables: [";
        for (int i = 0; i < new_xB.size(); ++i) {
            std::cout << std::fixed << std::setprecision(6) << new_xB(i);
            if (i < new_xB.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "  Objective value: " << std::fixed << std::setprecision(6) << objective_value << std::endl;
    } else if (iteration == 11) {
        std::cout << "\n  ... (showing only first 10 iterations in detail) ..." << std::endl;
    }
}

// ============================================================================
// Вывод итоговой сводки
// ============================================================================

void SimplexSolver2::print_final_summary(
    const Solution& sol,
    int phase1_iterations,
    int cleanup_iterations,
    int phase2_iterations,
    bool verbose) {

    if (!verbose) return;

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << " SIMPLEX METHOD COMPLETE: FINAL SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    std::cout << "\nIteration statistics:" << std::endl;
    std::cout << "  Phase I iterations:   " << phase1_iterations << std::endl;
    std::cout << "  Cleanup iterations:   " << cleanup_iterations << std::endl;
    std::cout << "  Phase II iterations:  " << phase2_iterations << std::endl;

    if (!sol.is_feasible) {
        std::cout << "\n>>> RESULT: INFEASIBLE PROBLEM <<<" << std::endl;
        std::cout << "  Status: " << sol.status_message << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        return;
    }

    std::cout << "\nOptimal solution found:" << std::endl;
    std::cout << "  Objective value: " << std::fixed << std::setprecision(6) << sol.objective_value << std::endl;

    std::cout << "  Solution vector (original variables): [";
    for (size_t i = 0; i < sol.x.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << sol.x[i];
        if (i < sol.x.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "  Basis indices in canonical form: {";
    for (size_t i = 0; i < sol.basis.size(); ++i) {
        std::cout << sol.basis[i];
        if (i < sol.basis.size() - 1) std::cout << ", ";
    }
    std::cout << "}" << std::endl;

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
    std::cout << std::string(70, '=') << std::endl;
}

// ============================================================================
// Обработка тривиальных случаев
// ============================================================================

SimplexSolver2::Solution SimplexSolver2::handle_no_constraints(size_t n, const std::vector<double>& c, bool verbose) {
    if (verbose) {
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << " TRIVIAL CASE: No constraints" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "Variables: " << n << std::endl;
    }

    bool unbounded = std::any_of(c.begin(), c.end(), [](double v) { return v < -TOLERANCE; });

    Solution sol;
    sol.x.resize(n, 0.0);
    sol.objective_value = unbounded ? -std::numeric_limits<double>::infinity() : 0.0;
    sol.basis.clear();
    sol.is_feasible = true;
    sol.is_optimal = !unbounded;
    sol.is_unbounded = unbounded;
    sol.is_degenerate = false;
    sol.status_message = unbounded ? "Unbounded problem (no constraints, negative cost coefficient)"
                                   : "Optimal solution at x=0";

    if (verbose) {
        std::cout << "Result: " << (unbounded ? "UNBOUNDED" : "OPTIMAL at x=0") << std::endl;
        std::cout << "Objective value: " << sol.objective_value << std::endl;
        std::cout << std::string(70, '=') << std::endl;
    }

    return sol;
}

SimplexSolver2::Solution SimplexSolver2::handle_no_variables(size_t m, const std::vector<double>& b, bool verbose) {
    if (verbose) {
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << " TRIVIAL CASE: No variables" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "Constraints: " << m << std::endl;
    }

    bool feasible = std::all_of(b.begin(), b.end(), [](double v) { return std::abs(v) < TOLERANCE; });

    Solution sol = create_infeasible_solution(0);
    if (feasible) {
        sol.is_feasible = true;
        sol.objective_value = 0.0;
        sol.status_message = "Feasible (0=0 constraints)";
    } else {
        sol.status_message = "Infeasible (0=b with b≠0)";
    }

    if (verbose) {
        std::cout << "Result: " << (feasible ? "FEASIBLE" : "INFEASIBLE") << std::endl;
        std::cout << "Status: " << sol.status_message << std::endl;
        std::cout << std::string(70, '=') << std::endl;
    }

    return sol;
}