#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "LinearProgram.hpp"

class EnumSolver {
public:
    /**
     * Структура решения задачи ЛП
     */
    struct Solution {
        std::vector<double> x; ///< Полный вектор решения (в исходных переменных)
        double objective_value; ///< Значение целевой функции
        std::vector<size_t> basis; ///< Индексы базисных переменных в канонической форме
        bool is_feasible; ///< Существует ли допустимое решение
        bool is_optimal; ///< Является ли решение оптимальным (проверка приведённых стоимостей)
        bool is_unbounded; ///< Неограниченность задачи
        bool is_degenerate;
        std::string status_message; ///< Текстовое описание статуса решения
    };

    /**
     * Решает задачу ЛП методом перебора всех опорных векторов (базисных решений)
     *
     * Алгоритм:
     * 1. Преобразует задачу в каноническую форму (min c^T x, Ax = b, x >= 0)
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
     * @return Оптимальное решение или недопустимое решение при отсутствии допустимой области
     */
    static Solution solve(const LinearProgram& lp, bool verbose = false);

private:
    static constexpr double TOLERANCE = 1e-9;

    static Solution create_infeasible_solution(int n);
    static std::vector<std::vector<size_t>> generate_combinations(int n, int k);
    static bool is_basis_feasible(const Eigen::VectorXd& xB, double tolerance);
    static bool is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance);

    static Solution evaluate_basis(const LinearProgram& canonical, const Eigen::VectorXd& c, const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& b, const std::vector<size_t>& basis, double tolerance);
    static void print_basis_info(const std::vector<size_t>& basis, const Eigen::VectorXd& xB, double obj_value,
                                 bool degenerate, int solution_index, bool is_new_best, bool verbose);
    static void print_final_summary(const Solution& best, size_t total_bases, int feasible_count, int degenerate_count,
                                    int singular_count, bool verbose);
    static std::vector<double> restore_original_solution(const LinearProgram& original_lp,
                                                         const std::vector<double>& canonical_solution);
    static bool check_optimality(const LinearProgram& canonical, const Eigen::VectorXd& c, const Eigen::MatrixXd& A,
                                 const Eigen::VectorXd& b, const std::vector<size_t>& basis, double tolerance,
                                 bool& is_unbounded);
};
