#include "SimplexSolver.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <numeric>
#include <stdexcept>

const double EPS = 1e-9;

// Вспомогательная функция: проверка ранга
size_t matrix_rank(const std::vector<std::vector<double>>& A) {
    if (A.empty() || A[0].empty())
        return 0;
    int m = A.size();
    int n = A[0].size();
    Eigen::MatrixXd M(m, n);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            M(i, j) = A[i][j];
    return M.fullPivLu().rank();
}

// Внутренняя структура результата симплекса
struct SimplexInternalResult {
    bool feasible = true;
    bool bounded = true;
    std::vector<size_t> basis;
    std::vector<double> x_b; // базисное решение
    double objective_value = 0.0;
};

// Реализация ревизионного симплекса (как в методичке)
SimplexInternalResult solve_simplex(const std::vector<double>& c, const std::vector<std::vector<double>>& A_mat,
                                    const std::vector<double>& b_vec, std::vector<size_t> basis, bool is_auxiliary) {
    const size_t m = A_mat.size();
    const size_t n_total = c.size();
    const size_t max_iter = 10000;

    Eigen::Map<const Eigen::VectorXd> b(b_vec.data(), m);
    Eigen::MatrixXd A(m, n_total);
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j < n_total; ++j)
            A(i, j) = A_mat[i][j];

    for (size_t iter = 0; iter < max_iter; ++iter) {
        // 1. Строим B и c_B
        Eigen::MatrixXd B(m, m);
        Eigen::VectorXd c_B(m);
        for (size_t i = 0; i < m; ++i) {
            size_t col = basis[i];
            B.col(i) = A.col(col);
            c_B(i) = c[col];
        }

        // 2. Вычисляем B^{-1}
        Eigen::MatrixXd B_inv;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(B);
        if (lu.rank() < m) {
            // Вырожденная базисная матрица — ошибка
            return SimplexInternalResult{.feasible = false};
        }
        B_inv = lu.inverse();

        // 3. x_B = B^{-1} b
        Eigen::VectorXd x_B = B_inv * b;

        // Проверка допустимости
        bool feasible = true;
        for (int i = 0; i < m; ++i) {
            if (x_B(i) < -EPS) {
                feasible = false;
                break;
            }
        }
        if (!feasible) {
            return SimplexInternalResult{.feasible = false};
        }

        // 4. y = c_B^T * B^{-1}
        Eigen::RowVectorXd y = c_B.transpose() * B_inv;

        // 5. Оценки d_j = c_j - y * A_j
        int entering = -1;
        for (size_t j = 0; j < n_total; ++j) {
            double d_j = c[j] - y.dot(A.col(j));
            if (d_j < -EPS) {
                entering = static_cast<int>(j);
                break; // правило Бленда: минимальный индекс
            }
        }

        if (entering == -1) {
            // Оптимум
            SimplexInternalResult res;
            res.feasible = true;
            res.basis = basis;
            res.x_b.resize(m);
            for (int i = 0; i < m; ++i)
                res.x_b[i] = x_B(i);
            res.objective_value = y.dot(b);
            return res;
        }

        // 6. Направление u = B^{-1} * A_entering
        Eigen::VectorXd u = B_inv * A.col(entering);

        // 7. Проверка неограниченности
        bool unbounded = true;
        for (int i = 0; i < m; ++i) {
            if (u(i) > EPS) {
                unbounded = false;
                break;
            }
        }
        if (unbounded) {
            SimplexInternalResult res;
            res.feasible = true;
            res.bounded = false;
            return res;
        }

        // 8. Выбор покидающей переменной (правило Бленда)
        double theta_min = std::numeric_limits<double>::max();
        int leaving = -1;
        for (int i = 0; i < m; ++i) {
            if (u(i) > EPS) {
                double theta = x_B(i) / u(i);
                if (theta < theta_min - EPS) {
                    theta_min = theta;
                    leaving = i;
                } else if (std::abs(theta - theta_min) < EPS) {
                    if (leaving == -1 || i < leaving) {
                        leaving = i; // правило Бленда
                    }
                }
            }
        }

        if (leaving == -1) {
            return SimplexInternalResult{.feasible = false};
        }

        // 9. Обновляем базис
        basis[leaving] = entering;
    }

    throw std::runtime_error("Simplex did not converge (cycling despite Blundell).");
}


SimplexSolver::Solution SimplexSolver::solve(const LinearProgram& lp) {
    if (lp.getForm() != LinearProgram::CANONIC) {
        throw std::runtime_error("Input must be in canonical form.");
    }

    const size_t n = lp.num_variables();
    const size_t m = lp.num_constraints();

    if (m == 0) {
        Solution sol;
        sol.feasible = true;
        sol.x.assign(n, 0.0);
        sol.objective_value = 0.0;
        return sol;
    }

    // Копируем данные
    std::vector<double> c = lp.objective();
    std::vector<std::vector<double>> A = lp.constraints();
    std::vector<double> b = lp.rhs();

    // Приводим b >= 0
    for (size_t i = 0; i < m; ++i) {
        if (b[i] < 0) {
            b[i] = -b[i];
            for (auto& a : A[i])
                a = -a;
        }
    }

    // Проверка ранга
    if (matrix_rank(A) < m) {
        throw std::runtime_error("Constraint matrix rank < number of constraints.");
    }

    // === ШАГ 1: ВСПОМОГАТЕЛЬНАЯ ЗАДАЧА ===
    const size_t n_aux = n + m;
    std::vector<size_t> basis(m);
    std::iota(basis.begin(), basis.end(), n); // w1..wm

    // Целевая функция вспомогательной задачи: min sum(w_i)
    std::vector<double> c_aux(n_aux, 0.0);
    for (size_t i = n; i < n_aux; ++i)
        c_aux[i] = 1.0;

    // Матрица A_aux = [A | I]
    std::vector<std::vector<double>> A_aux(m, std::vector<double>(n_aux, 0.0));
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j)
            A_aux[i][j] = A[i][j];
        A_aux[i][n + i] = 1.0;
    }

    // Решаем вспомогательную задачу
    auto aux_sol = solve_simplex(c_aux, A_aux, b, basis, true);

    if (!aux_sol.feasible || aux_sol.objective_value > EPS) {
        return Solution{}; // несовместна
    }

    // === ШАГ 2: УДАЛЕНИЕ ИСКУССТВЕННЫХ ПЕРЕМЕННЫХ ИЗ БАЗИСА ===
    std::vector<size_t> new_basis;
    std::vector<bool> in_new_basis(n_aux, false);

    // Сначала берём все исходные переменные из базиса
    for (size_t idx : aux_sol.basis) {
        if (idx < n) {
            new_basis.push_back(idx);
            in_new_basis[idx] = true;
        }
    }

    // Добавляем недостающие исходные переменные
    for (size_t j = 0; j < n && new_basis.size() < m; ++j) {
        if (!in_new_basis[j]) {
            new_basis.push_back(j);
            in_new_basis[j] = true;
        }
    }

    if (new_basis.size() != m) {
        throw std::runtime_error("Failed to construct initial basis without artificial variables.");
    }

    // === ШАГ 3: ОСНОВНАЯ ЗАДАЧА ===
    auto main_sol = solve_simplex(c, A, b, new_basis, false);

    if (!main_sol.feasible) {
        return Solution{};
    }

    Solution result;
    result.feasible = true;
    result.bounded = main_sol.bounded;
    result.x.assign(n, 0.0);
    for (size_t i = 0; i < m; ++i) {
        size_t var = main_sol.basis[i];
        if (var < n) {
            result.x[var] = main_sol.x_b[i];
        }
    }
    result.objective_value = 0.0;
    for (size_t i = 0; i < n; ++i) {
        result.objective_value += c[i] * result.x[i];
    }
    return result;
}