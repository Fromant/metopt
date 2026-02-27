#pragma once

#include <string>
#include <vector>

#include "../linear/LinearProgram.hpp"

class TransportProblem {
public:
    std::vector<double> supply; // Запасы поставщиков (a_i)
    std::vector<double> demand; // Потребности потребителей (b_j)
    std::vector<std::vector<double>> cost; // Матрица стоимостей (c_ij)
    std::vector<double> demandPenalty; // Штраф за недопоставку для каждого потребителя

    size_t m; // Количество поставщиков
    size_t n; // Количество потребителей

    TransportProblem() : m(0), n(0) {}

    // Чтение из файла (формат: m n, затем a_1..a_m, затем b_1..b_n, затем матрица cost)
    static TransportProblem fromFile(const std::string& filename);

    // Чтение из консоли
    static TransportProblem fromConsole();

    // Проверка корректности задачи
    bool validate() const;

    // Проверка сбалансированности
    bool isBalanced(double eps = 1e-9) const;

    // Балансировка (добавление фиктивного поставщика/потребителя)
    TransportProblem balance() const;

    // Преобразование в задачу ЛП для симплекс-метода
    LinearProgram toLinearProgram() const;

    // Вывод задачи
    void print(const std::string& title = "Transport Problem") const;

    // Вывод плана перевозок
    static void printPlan(const std::vector<std::vector<double>>& plan);

    // Подсчёт общей стоимости
    double calculateCost(const std::vector<std::vector<double>>& plan) const;

    /**
     * Восстановить матрицу плана из вектора решения (формат: x[i*n + j])
     */
    static std::vector<std::vector<double>> restorePlanFromVector(const std::vector<double>& x_vector, size_t m,
                                                                  size_t n);
};
