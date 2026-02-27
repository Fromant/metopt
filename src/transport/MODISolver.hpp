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
    
private:
    // Вычисление потенциалов
    static bool computePotentials(const TransportProblem& problem,
                                 const std::vector<std::pair<size_t, size_t>>& basis,
                                 std::vector<double>& u,
                                 std::vector<double>& v);
    
    // Вычисление оценки для свободной клетки
    static double computeDelta(const TransportProblem& problem,
                              size_t i, size_t j,
                              const std::vector<double>& u,
                              const std::vector<double>& v);
    
    // Поиск клетки с минимальной оценкой
    static std::optional<std::pair<size_t, size_t>> findEnteringCell(
        const TransportProblem& problem,
        const std::vector<std::pair<size_t, size_t>>& basis,
        const std::vector<double>& u,
        const std::vector<double>& v);
    
    // Построение цикла
    static std::vector<std::pair<size_t, size_t>> findCycle(
        size_t startI, size_t startJ,
        const std::vector<std::pair<size_t, size_t>>& basis);
    
    // Применение цикла
    static void applyCycle(std::vector<std::vector<double>>& plan,
                          const std::vector<std::pair<size_t, size_t>>& cycle,
                          size_t& leavingIndex);
    
    // Вспомогательные методы вывода
    static void printPotentials(const std::vector<double>& u, 
                               const std::vector<double>& v);
    static void printDeltas(const TransportProblem& problem,
                           const std::vector<std::pair<size_t, size_t>>& basis,
                           const std::vector<double>& u,
                           const std::vector<double>& v);
    static void printCycle(const std::vector<std::pair<size_t, size_t>>& cycle,
                          const std::vector<std::vector<double>>& plan);
};