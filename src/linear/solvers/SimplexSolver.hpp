#pragma once

#include <Eigen/Dense>

#include <optional>
#include <string>
#include <vector>

#include "linear/LinearProgram.hpp"

class SimplexSolver {
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

    /// Решает задачу линейного программирования симплекс-методом
    /// @param lp Исходная задача (любая форма)
    /// @param verbose Выводить ли подробную информацию о ходе решения
    /// @return Решение задачи
    static Solution solve(const LinearProgram& lp, bool verbose = false);

private:
    static constexpr double EPS = 1e-9;
    static constexpr int MAX_ITERATIONS = 1000;

    static Solution create_infeasible_solution(int n);

    static bool is_basis_feasible(const Eigen::VectorXd& xB, double tolerance);

    static bool is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance);

    struct PhaseResult {
        bool optimal;
        bool unbounded;
        int iterations;
        Eigen::MatrixXd B_inv;
        Eigen::VectorXd x_B;
        std::vector<size_t> basis;
        double objective_value;
    };
    /// Единый движок симплекс-метода для обеих фаз
    /// @param A Матрица ограничений (m * n_active)
    /// @param b Вектор правой части (m)
    /// @param c Коэффициенты целевой функции (n_active)
    /// @param basis Текущий базис (размер m)
    /// @param B_inv Обратная базисная матрица (m * m)
    /// @param x_B Базисное решение (m)
    /// @param n_active_vars Число активных переменных
    /// @param phase_name Имя фазы для логирования
    /// @param verbose Режим подробного вывода
    /// @return Результат итераций или std::nullopt при ошибке
    static std::optional<PhaseResult> run_simplex_phase(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                                        const std::vector<double>& c, std::vector<size_t> basis,
                                                        Eigen::MatrixXd B_inv, Eigen::VectorXd x_B,
                                                        size_t n_active_vars, const std::string& phase_name,
                                                        bool verbose);

    /// Удаляет искусственные переменные из базиса с помощью вырожденных замен
    /// @return true если все искусственные переменные успешно удалены
    static bool cleanup_artificial_basis(const Eigen::MatrixXd& A_orig, Eigen::MatrixXd& B_inv,
                                         std::vector<size_t>& basis, Eigen::VectorXd& x_B, size_t n_orig, bool verbose);

    static void print_basis_change(const std::string& phase_name, int iteration, size_t entering_var,
                                   size_t leaving_var, size_t leaving_pos, double reduced_cost, double theta,
                                   bool degenerate, const std::vector<size_t>& new_basis, const Eigen::VectorXd& new_xB,
                                   double objective_value, bool verbose);

    static void print_final_summary(const Solution& sol, int phase1_iterations, int phase2_iterations, bool verbose);

    static Solution handle_no_constraints(size_t n, const std::vector<double>& c, bool verbose);
    static Solution handle_no_variables(size_t m, const std::vector<double>& b, bool verbose);

    static bool check_optimality(const Eigen::MatrixXd& A, const Eigen::VectorXd& c, const std::vector<size_t>& basis,
                                 const Eigen::MatrixXd& B_inv, bool& is_unbounded);
};
