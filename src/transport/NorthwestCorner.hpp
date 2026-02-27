#pragma once

#include <vector>
#include "TransportProblem.hpp"

class NorthwestCorner {
public:
    // Структура результата
    struct Result {
        std::vector<std::vector<double>> plan; // План перевозок
        std::vector<std::pair<size_t, size_t>> basis; // Индексы базисных клеток
        double totalCost;
        bool success;
        std::string errorMessage;
    };

    // Основной метод
    static Result solve(const TransportProblem& problem, bool verbose = true);

private:
    // Вспомогательные методы
    static void printStep(size_t step, size_t i, size_t j, double value, double remainingSupply, double remainingDemand,
                          const std::vector<std::vector<double>>& currentPlan);
};
