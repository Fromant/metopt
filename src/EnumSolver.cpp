#include "EnumSolver.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>

#include "FormConverter.hpp"

EnumSolver::Solution EnumSolver::create_infeasible_solution(int n) {
    Solution sol;
    sol.x.resize(n, 0.0);
    sol.objective_value = std::numeric_limits<double>::infinity();
    sol.is_feasible = false;
    sol.is_optimal = false;
    sol.is_unbounded = false;
    sol.is_degenerate = false;
    sol.status_message = "No feasible solution exists (empty feasible region)";
    return sol;
}

/// Generates all C(n, k) combinations
///
/// generate_combinations(5, 3) =
/// {0,1,2}, {0,1,3}, {0,1,4}, {0,2,3}, {0,2,4},
/// {0, 3, 4}, {1, 2, 3}, {1, 2, 4}, {1, 3, 4}, {2, 3, 4}
std::vector<std::vector<size_t>> EnumSolver::generate_combinations(int n, int k) {
    std::vector<std::vector<size_t>> result;
    if (k > n || k <= 0)
        return result;

    std::vector<size_t> combination(k);
    std::iota(combination.begin(), combination.end(), 0);

    while (true) {
        result.push_back(combination);

        int i = k - 1;
        while (i >= 0 && combination[i] == static_cast<size_t>(n - k + i)) {
            i--;
        }

        if (i < 0) {
            break; // All combinations generated
        }

        // Increment and reset subsequent elements
        combination[i]++;
        for (int j = i + 1; j < k; ++j) {
            combination[j] = combination[j - 1] + 1;
        }
    }

    return result;
}

/// Проверяет базис на допустимость.
///
/// Условия допустимости: Ax=B, x>=0
///
/// Эта функция проверяет x>=0
/// @returns Допустим ли базис
bool EnumSolver::is_basis_feasible(const Eigen::VectorXd& xB, double tolerance) {
    return !std::ranges::any_of(xB, [tolerance](const double d) { return d < -tolerance; });
}

/// Проверяет базис на вырожденность.
/// Условия вырожденности: хотя бы одна базисная переменная равна нулю
/// @returns Вырожден ли базис
bool EnumSolver::is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance) {
    return std::ranges::any_of(xB, [tolerance](const double d) { return std::abs(d) < tolerance; });
}

void EnumSolver::print_basis_info(const std::vector<size_t>& basis, const Eigen::VectorXd& xB, double obj_value,
                                  bool degenerate, int solution_index, bool is_new_best, bool verbose) {
    if (!verbose)
        return;

    // Show detailed info for first 10 solutions or when new best found
    if (solution_index <= 10 || is_new_best) {
        std::cout << "\n--- Feasible Basic Solution #" << solution_index << " ---" << std::endl;

        // Print basis indices
        std::cout << "  Basis indices (B): {";
        for (size_t i = 0; i < basis.size(); ++i) {
            std::cout << basis[i];
            if (i < basis.size() - 1)
                std::cout << ", ";
        }
        std::cout << "}" << std::endl;

        // Print basic variable values
        std::cout << "  Basic variables (x_B): [";
        for (int i = 0; i < xB.size(); ++i) {
            std::cout << std::fixed << std::setprecision(6) << xB(i);
            if (i < xB.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Highlight degeneracy
        if (degenerate) {
            std::cout << "  *** DEGENERATE SOLUTION *** (some x_B approx 0)" << std::endl;
        }

        // Print objective value
        std::cout << "  Objective value: " << std::fixed << std::setprecision(6) << obj_value << std::endl;

        // Highlight new best solution
        if (is_new_best) {
            std::cout << "  >>> NEW BEST SOLUTION <<<" << std::endl;
        }
    } else if (solution_index == 11) {
        std::cout << "\n  ... (showing only first 10 feasible solutions in detail) ..." << std::endl;
    }
}

void EnumSolver::print_final_summary(const Solution& best, size_t total_bases, int feasible_count, int degenerate_count,
                                     int singular_count, bool verbose) {
    if (!verbose)
        return;

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << " ENUMERATION COMPLETE: FINAL SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    std::cout << "\nEnumeration statistics:" << std::endl;
    std::cout << "  Total bases examined:       " << total_bases << std::endl;
    std::cout << "  Singular basis matrices:    " << singular_count << std::endl;
    std::cout << "  Feasible basic solutions:   " << feasible_count << std::endl;
    std::cout << "  Degenerate solutions:       " << degenerate_count << std::endl;

    if (!best.is_feasible) {
        std::cout << "\n>>> RESULT: INFEASIBLE PROBLEM <<<" << std::endl;
        std::cout << "  Status: " << best.status_message << std::endl;
        return;
    }

    std::cout << "\nOptimal solution found:" << std::endl;
    std::cout << "  Objective value: " << std::fixed << std::setprecision(6) << best.objective_value << std::endl;

    std::cout << "  Solution vector (original variables): [";
    for (size_t i = 0; i < best.x.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << best.x[i];
        if (i < best.x.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "  Basis indices in canonical form: {";
    for (size_t i = 0; i < best.basis.size(); ++i) {
        std::cout << best.basis[i];
        if (i < best.basis.size() - 1)
            std::cout << ", ";
    }
    std::cout << "}" << std::endl;

    if (best.is_unbounded) {
        std::cout << "\n  *** WARNING: PROBLEM IS UNBOUNDED ***" << std::endl;
        std::cout << "      Objective can be decreased indefinitely while maintaining feasibility." << std::endl;
    } else if (best.is_optimal) {
        std::cout << "\n  *** SOLUTION IS OPTIMAL ***" << std::endl;
        std::cout << "      All reduced costs are non-negative (minimization problem)." << std::endl;
    } else {
        std::cout << "\n  *** WARNING: SOLUTION MAY NOT BE OPTIMAL ***" << std::endl;
        std::cout << "      Some reduced costs are negative. This may indicate:" << std::endl;
        std::cout << "        - Numerical issues during enumeration" << std::endl;
        std::cout << "        - Problem has no optimal solution (unbounded)" << std::endl;
        std::cout << "        - Enumeration missed optimal basis due to singularity tolerance" << std::endl;
    }

    std::cout << "\nStatus: " << best.status_message << std::endl;
    std::cout << std::string(70, '=') << std::endl;
}

// ============================================================================
// Восстановление решения исходной задачи из канонической формы
// ============================================================================

std::vector<double> EnumSolver::restore_original_solution(const LinearProgram& original_lp,
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
// Проверка оптимальности через приведённые стоимости
// ============================================================================
bool EnumSolver::check_optimality(const LinearProgram& canonical, const Eigen::VectorXd& c, const Eigen::MatrixXd& A,
                                  const Eigen::VectorXd& b, const std::vector<size_t>& basis, double tolerance,
                                  bool& is_unbounded) {
    is_unbounded = false;
    const int m = static_cast<int>(basis.size());
    const int n = static_cast<int>(c.size());

    // Извлекаем базисную матрицу B (m x m) — СТОЛБЦЫ из базиса
    Eigen::MatrixXd B(m, m);
    Eigen::VectorXd cB(m);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            B(i, j) = A(i, static_cast<int>(basis[j]));
        }
        // Коэффициент целевой функции для i-й базисной переменной
        cB(i) = c(static_cast<int>(basis[i]));
    }

    // Вычисляем обратную матрицу базиса
    Eigen::FullPivLU<Eigen::MatrixXd> lu(B);
    if (lu.rank() < m)
        return false; // Вырожденный базис

    Eigen::MatrixXd B_inv = lu.inverse();

    // Вычисляем двойственные переменные: y = (B^{-1})^T * c_B
    Eigen::VectorXd y = B_inv.transpose() * cB;

    // Проверяем приведённые стоимости для всех небазисных переменных
    for (int j = 0; j < n; ++j) {
        // Пропускаем базисные переменные
        if (std::ranges::find(basis, j) != basis.end()) {
            continue;
        }

        // Приведённая стоимость: d_j = c_j - y^T * A_j
        double reduced_cost = c(j) - y.dot(A.col(j));

        // Для задачи минимизации оптимальность достигается при d_j >= 0
        if (reduced_cost < -tolerance) {
            // Проверяем на неограниченность: можно ли увеличивать x_j бесконечно?
            Eigen::VectorXd direction = B_inv * A.col(j);

            // Если все компоненты направления <= 0, то можем увеличивать x_j до бесконечности
            bool can_increase_indefinitely = true;
            for (int i = 0; i < m; ++i) {
                if (direction(i) > tolerance) {
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

// ============================================================================
// Оценка конкретного базиса
// ============================================================================

EnumSolver::Solution EnumSolver::evaluate_basis(const LinearProgram& canonical, const Eigen::VectorXd& c,
                                                const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                                const std::vector<size_t>& basis, double tolerance) {
    const int m = static_cast<int>(basis.size());
    const int n = static_cast<int>(c.size());

    // Извлекаем базисную матрицу B (m x m)
    Eigen::MatrixXd B(m, m);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            B(i, j) = A(i, static_cast<int>(basis[j]));
        }
    }

    // Проверяем невырожденность базиса через ранг матрицы
    Eigen::FullPivLU<Eigen::MatrixXd> lu(B);
    if (lu.rank() < m || std::abs(lu.determinant()) < tolerance) {
        Solution sol;
        sol.is_feasible = false;
        sol.status_message = "Singular basis matrix (rank deficient)";
        return sol;
    }

    // Вычисляем базисное решение: x_B = B^{-1} * b
    Eigen::VectorXd xB;
    try {
        xB = lu.solve(b);
    } catch (...) {
        Solution sol;
        sol.is_feasible = false;
        sol.status_message = "LU decomposition failed";
        return sol;
    }

    // Проверяем допустимость базисного решения
    if (!is_basis_feasible(xB, tolerance)) {
        Solution sol;
        sol.is_feasible = false;
        sol.status_message = "Infeasible basis (negative basic variables)";
        return sol;
    }

    // Конструируем полное решение в канонической форме
    Eigen::VectorXd x_full(n);
    x_full.setZero();
    for (int i = 0; i < m; ++i) {
        x_full(static_cast<int>(basis[i])) = xB(i);
    }

    // Вычисляем значение целевой функции
    double obj_value = c.dot(x_full);

    // Формируем решение
    Solution sol;
    sol.x.resize(n);
    for (int i = 0; i < n; ++i) {
        sol.x[i] = x_full(i);
    }
    sol.objective_value = obj_value;
    sol.basis = basis;
    sol.is_feasible = true;
    sol.is_degenerate = is_basis_degenerate(xB, tolerance);
    sol.status_message = sol.is_degenerate ? "Degenerate feasible solution" : "Feasible solution";

    return sol;
}

// ============================================================================
// Основной метод решения
// ============================================================================

EnumSolver::Solution EnumSolver::solve(const LinearProgram& lp, bool verbose) {
    if (verbose) {
        lp.print("Enumeration solver. Original problem:");
    }

    LinearProgram canonical = FormConverter::to_canonical_form(lp);

    if (verbose && lp.getForm() != LinearProgram::CANONIC) {
        canonical.print("Problem in canonical form: ");
    }

    // Извлекаем размерности
    int n = static_cast<int>(canonical.num_variables());
    int m = static_cast<int>(canonical.num_constraints());

    // Преобразуем данные в структуры Eigen
    Eigen::VectorXd c(n);
    for (int i = 0; i < n; ++i)
        c(i) = canonical.objective()[i];

    Eigen::MatrixXd A(m, n);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            A(i, j) = canonical.constraints()[i][j];
        }
    }

    Eigen::VectorXd b(m);
    for (int i = 0; i < m; ++i)
        b(i) = canonical.rhs()[i];

    // Обеспечиваем неотрицательность правой части (требование канонической формы)
    for (int i = 0; i < m; ++i) {
        if (b(i) < -TOLERANCE) {
            if (verbose) {
                std::cout << "\nWARNING: Negative RHS component b[" << i << "] = " << b(i)
                          << ". Multiplying constraint by -1." << std::endl;
            }
            A.row(i) = -A.row(i);
            b(i) = -b(i);
        }
    }

    // Шаг 2: Генерируем все возможные базисы
    auto bases = generate_combinations(n, m);
    size_t total_bases = bases.size();

    // Инициализируем лучшее решение
    Solution best = create_infeasible_solution(n);
    best.objective_value = std::numeric_limits<double>::infinity();

    int feasible_count = 0;
    int degenerate_count = 0;
    int singular_count = 0;

    // Шаг 3: Перебираем все базисы
    if (verbose) {
        std::cout << "\n[Step 3] Enumerating all basic feasible solutions..." << std::endl;
        std::cout << std::string(70, '-') << std::endl;
    }

    for (size_t idx = 0; idx < total_bases; ++idx) {
        // Оцениваем текущий базис
        Solution sol = evaluate_basis(canonical, c, A, b, bases[idx], TOLERANCE);

        if (!sol.is_feasible) {
            if (sol.status_message.find("Singular") != std::string::npos) {
                singular_count++;
            }
            continue;
        }

        feasible_count++;
        if (sol.is_degenerate)
            degenerate_count++;

        // Выводим информацию о допустимом решении
        print_basis_info(bases[idx], Eigen::VectorXd::Map(sol.x.data(), sol.x.size()), sol.objective_value,
                         sol.is_degenerate, feasible_count, sol.objective_value < best.objective_value - TOLERANCE,
                         verbose && feasible_count <= 10);

        // Обновляем лучшее решение
        if (sol.objective_value < best.objective_value - TOLERANCE) {
            best = sol;
            best.is_feasible = true;
        }
    }

    // Шаг 4: Анализ результатов
    if (!best.is_feasible) {
        best = create_infeasible_solution(lp.num_variables());
        print_final_summary(best, total_bases, feasible_count, degenerate_count, singular_count, verbose);
        return best;
    }

    // Проверка оптимальности через приведённые стоимости
    bool is_unbounded = false;
    best.is_optimal = check_optimality(canonical, c, A, b, best.basis, TOLERANCE, is_unbounded);
    best.is_unbounded = is_unbounded;

    if (is_unbounded) {
        best.status_message = "Unbounded problem (objective can be decreased indefinitely)";
    } else if (best.is_optimal) {
        best.status_message = "Optimal solution found";
    } else {
        best.status_message = "Feasible solution found (may not be optimal due to numerical issues)";
    }

    // Восстанавливаем решение исходной задачи
    best.x = restore_original_solution(lp, best.x);

    // Корректируем знак целевой функции если исходная задача была на максимизацию
    if (!lp.is_minimization()) {
        best.objective_value = -best.objective_value;
    }

    print_final_summary(best, total_bases, feasible_count, degenerate_count, singular_count, verbose);

    return best;
}
