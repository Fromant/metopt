#pragma once

#include <Eigen/Dense>

#include <optional>
#include <string>
#include <vector>

class LinearProgram;

class SimplexSolver2 {
public:
    struct Solution {
        std::vector<double> x;              // Solution vector (original variables)
        double objective_value;             // Objective function value
        std::vector<size_t> basis;          // Basis indices in canonical form
        bool is_feasible;                   // Feasibility flag
        bool is_optimal;                    // Optimality flag
        bool is_unbounded;                  // Unboundedness flag
        bool is_degenerate;                 // Degeneracy flag (any basic variable ≈ 0)
        std::string status_message;         // Human-readable status
    };

    /// Решает задачу линейного программирования симплекс-методом
    /// @param lp Исходная задача (любая форма)
    /// @param verbose Выводить ли подробную информацию о ходе решения
    /// @return Решение задачи
    static Solution solve(const LinearProgram& lp, bool verbose = false
        );

    /// Восстанавливает решение исходной задачи из решения в канонической форме
    static std::vector<double> restore_original_solution(const LinearProgram& original_lp,
                                                         const std::vector<double>& canonical_solution);

private:
    static constexpr double TOLERANCE = 1e-9;
    static constexpr int MAX_ITERATIONS = 1000;

    /// Создаёт решение для недопустимой задачи
    static Solution create_infeasible_solution(int n);

    /// Проверяет базис на допустимость (все базисные переменные >= 0)
    static bool is_basis_feasible(const Eigen::VectorXd& xB, double tolerance);

    /// Проверяет базис на вырожденность (хотя бы одна базисная переменная ≈ 0)
    static bool is_basis_degenerate(const Eigen::VectorXd& xB, double tolerance);

    /// Единый движок симплекс-метода для обеих фаз
    /// @param A Матрица ограничений (m × n_active)
    /// @param b Вектор правой части (m)
    /// @param c Коэффициенты целевой функции (n_active)
    /// @param basis Текущий базис (размер m)
    /// @param B_inv Обратная базисная матрица (m × m)
    /// @param x_B Базисное решение (m)
    /// @param n_active_vars Число активных переменных (для фазы I = n_orig, для фазы II = n_orig)
    /// @param phase_name Имя фазы для логгирования ("Phase I", "Phase II")
    /// @param verbose Режим подробного вывода
    /// @return Результат итераций или std::nullopt при ошибке
    struct PhaseResult {
        bool optimal;
        bool unbounded;
        int iterations;
        Eigen::MatrixXd B_inv;
        Eigen::VectorXd x_B;
        std::vector<size_t> basis;
        double objective_value;
    };
    static std::optional<PhaseResult> run_simplex_phase(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        const std::vector<double>& c,
        std::vector<size_t> basis,
        Eigen::MatrixXd B_inv,
        Eigen::VectorXd x_B,
        size_t n_active_vars,
        const std::string& phase_name,
        bool verbose);

    /// Удаляет искусственные переменные из базиса с помощью вырожденных замен
    /// @return true если все искусственные переменные успешно удалены
    static bool cleanup_artificial_basis(
        const Eigen::MatrixXd& A_orig,
        Eigen::MatrixXd& B_inv,
        std::vector<size_t>& basis,
        Eigen::VectorXd& x_B,
        size_t n_orig,
        bool verbose);

    /// Выводит информацию о смене базиса
    static void print_basis_change(
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
        bool verbose);

    /// Выводит итоговую сводку решения
    static void print_final_summary(
        const Solution& sol,
        int phase1_iterations,
        int cleanup_iterations,
        int phase2_iterations,
        bool verbose);

    /// Обработка тривиальных случаев (нет ограничений / нет переменных)
    static Solution handle_no_constraints(size_t n, const std::vector<double>& c, bool verbose);
    static Solution handle_no_variables(size_t m, const std::vector<double>& b, bool verbose);
};