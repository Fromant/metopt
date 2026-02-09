#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "LinearProgram.hpp"

class SimplexSolver {
public:
    struct Solution {
        std::vector<double> x;
        double objective_value;
        bool is_unbounded;
        bool is_infeasible;
        int iterations;
        std::string status;
    };

    // Основной метод решения
    static std::optional<Solution> solve(const LinearProgram& lp, bool verbose = false);

private:
    // Состояние симплекс-метода (сохраняем ОБРАТНУЮ матрицу базиса B⁻¹!)
    struct SimplexState {
        Eigen::MatrixXd A;          // Матрица ограничений (исходная, без расширения)
        Eigen::VectorXd b;          // Вектор правых частей (≥ 0)
        Eigen::VectorXd c;          // Коэффициенты целевой функции
        std::vector<int> basis;     // Индексы базисных переменных (размер m)
        Eigen::VectorXd x_B;        // Значения базисных переменных
        Eigen::MatrixXd B_inv;      // ОБРАТНАЯ матрица базиса B⁻¹ (формула 5.8-5.10!)
        int m;                      // Число ограничений
        int n;                      // Число исходных переменных
    };

    // === ФАЗА I: Искусственный базис (стр. 25-26 методички) ===
    static std::optional<SimplexState> phase1(const LinearProgram& lp, bool verbose);

    // === ФАЗА II: Основной симплекс-метод (стр. 22-25 методички) ===
    static Solution phase2(SimplexState state, const LinearProgram& lp, bool verbose);

    // === ШАГИ СИМПЛЕКС-МЕТОДА (точное соответствие уравнениям методички) ===

    // Шаг 1: Вычисление двойственных переменных y = c_B · B⁻¹ (уравнение 5.2)
    static Eigen::VectorXd step1_compute_dual_vars(
        const Eigen::VectorXd& c_B,
        const Eigen::MatrixXd& B_inv
    );

    // Шаг 2: Вычисление приведённых стоимостей d = c - y·A (уравнения 5.3-5.5)
    static Eigen::VectorXd step2_compute_reduced_costs(
        const Eigen::VectorXd& c,
        const Eigen::VectorXd& y,
        const Eigen::MatrixXd& A
    );

    // Шаг 3: Проверка оптимальности (условие 4.20: d[L] ≥ 0?)
    static std::pair<bool, int> step3_check_optimality(
        const Eigen::VectorXd& d,
        const std::vector<int>& basis,
        int n_vars
    );

    // Шаг 4: Вычисление направления спуска u = B⁻¹ · a_j (перед уравнением 5.7)
    static Eigen::VectorXd step4_compute_direction(
        const Eigen::MatrixXd& B_inv,
        const Eigen::VectorXd& a_j
    );

    // Шаг 5: Проверка неограниченности (условие "u ≤ 0" → задача неограничена)
    static bool step5_check_unboundedness(const Eigen::VectorXd& u);

    // Шаг 6: Выбор выходящей переменной и шага θ (формула 5.6)
    static std::pair<int, double> step6_select_leaving_variable(
        const Eigen::VectorXd& x_B,
        const Eigen::VectorXd& u
    );

    // Шаг 7: Обновление решения x_new = x_old - θ·u (уравнение 5.7)
    static Eigen::VectorXd step7_update_solution(
        const Eigen::VectorXd& x_B_old,
        const Eigen::VectorXd& u,
        double theta,
        int leaving_idx
    );

    // Шаг 8: ОБНОВЛЕНИЕ ОБРАТНОЙ МАТРИЦЫ БАЗИСА (формулы 5.8-5.10!) — КЛЮЧЕВОЙ ШАГ
    static Eigen::MatrixXd step8_update_basis_inverse(
        const Eigen::MatrixXd& B_inv_old,
        const Eigen::VectorXd& u,
        int leaving_idx
    );

    // Вспомогательные методы
    static void prepare_rhs_nonnegative(
        Eigen::MatrixXd& A,
        Eigen::VectorXd& b,
        const LinearProgram& lp
    );

    static void remove_artificial_vars(
        SimplexState& state,
        int original_n,
        bool verbose
    );

    static void print_iteration(
        int iter,
        int entering,
        int leaving_var,
        int leaving_idx,
        double theta,
        const std::vector<int>& basis,
        const Eigen::VectorXd& x_B,
        double reduced_cost,
        bool degenerate,
        bool verbose
    );
};