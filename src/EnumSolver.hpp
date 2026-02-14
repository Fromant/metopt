#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "LinearProgram.hpp"

class EnumSolver {
public:
    struct Solution {
        std::vector<double> x;
        double objective_value;
        std::vector<size_t> basis;
        bool is_feasible;
        bool is_optimal;
        bool is_unbounded;
        bool is_degenerate;
        std::string status_message;
    };

    /**
     * Решает задачу ЛП методом перебора всех опорных векторов (базисных решений)
     *
     * Алгоритм:
     * 1. Преобразует задачу в каноническую форму
     * 2. Перебирает все C(n, m) возможных базисов
     * 3. Для каждого базиса:
     *    - Проверяет линейную независимость столбцов (ранг матрицы)
     *    - Вычисляет базисное решение x_B = B^{-1} b
     *    - Проверяет допустимость (x_B >= 0)
     *    - Вычисляет значение целевой функции
     * 4. Выбирает базис с минимальным значением целевой функции
     * 5. Восстанавливает решение исходной задачи из решения канонической формы
     *
     * @param lp Исходная задача линейного программирования
     * @param verbose Включить детальный вывод промежуточных результатов
     * @return Решение
     */
    static Solution solve(const LinearProgram& lp, bool verbose = false);

private:
    static constexpr double EPS = 1e-9;

    static Solution create_infeasible_solution(int n);
    static std::vector<std::vector<size_t>> generate_combinations(int n, int k);
    static bool is_basis_feasible(const Eigen::VectorXd& xB, double tolerance);
    static bool is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance);

    static Solution evaluate_basis(const Eigen::VectorXd& c, const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                   const std::vector<size_t>& basis, double tolerance);
    static void print_basis_info(const std::vector<size_t>& basis, const Eigen::VectorXd& xB, double obj_value,
                                 bool degenerate, int solution_index, bool is_new_best, bool verbose);
    static void print_final_summary(const Solution& best, size_t total_bases, int feasible_count, int degenerate_count,
                                    int singular_count, bool verbose);
    static bool check_optimality(const Eigen::VectorXd& c, const Eigen::MatrixXd& A, const std::vector<size_t>& basis,
                                 bool& is_unbounded);
};
