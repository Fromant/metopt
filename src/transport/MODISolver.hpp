#pragma once

#include "TransportProblem.hpp"
#include <vector>
#include <optional>

class MODISolver {
public:
    // Структура результата
    struct Result {
        std::vector<std::vector<double>> plan;  // Оптимальный план
        std::vector<std::pair<size_t, size_t>> basis;  // Базисные клетки
        double totalCost;
        int iterations;
        bool isOptimal;
        bool success;
        std::string errorMessage;
    };
    
    // Основной метод
    static Result solve(const TransportProblem& problem, 
                       const std::vector<std::vector<double>>& initialPlan,
                       const std::vector<std::pair<size_t, size_t>>& initialBasis,
                       bool verbose = true,
                       int maxIterations = 100);

    // Построение цикла (публичный метод для демонстрации)
    static std::vector<std::pair<size_t, size_t>> findCycle(
        size_t startI, size_t startJ,
        const std::vector<std::pair<size_t, size_t>>& basis);

    // Применение цикла (публичный метод для демонстрации)
    static void applyCycle(std::vector<std::vector<double>>& plan,
                          const std::vector<std::pair<size_t, size_t>>& cycle,
                          size_t& leavingIndex);

    // Вывод цикла в консоль (для демонстрации)
    static void printCycle(const std::vector<std::pair<size_t, size_t>>& cycle,
                          const std::vector<std::vector<double>>& plan);

    // Вывод потенциалов в консоль
    static void printPotentials(const std::vector<double>& u, 
                               const std::vector<double>& v);

    // Вывод оценок (delta) в консоль
    static void printDeltas(const TransportProblem& problem,
                           const std::vector<std::pair<size_t, size_t>>& basis,
                           const std::vector<double>& u,
                           const std::vector<double>& v);

    // Вычисление потенциалов (публичный метод для демонстрации)
    static bool computePotentials(const TransportProblem& problem,
                                 const std::vector<std::pair<size_t, size_t>>& basis,
                                 std::vector<double>& u,
                                 std::vector<double>& v);

    // Поиск клетки с минимальной оценкой (публичный метод для демонстрации)
    static std::optional<std::pair<size_t, size_t>> findEnteringCell(
        const TransportProblem& problem,
        const std::vector<std::pair<size_t, size_t>>& basis,
        const std::vector<double>& u,
        const std::vector<double>& v);

    // Вычисление оценки для свободной клетки
    static double computeDelta(const TransportProblem& problem,
                              size_t i, size_t j,
                              const std::vector<double>& u,
                              const std::vector<double>& v);
};