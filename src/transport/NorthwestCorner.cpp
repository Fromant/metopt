#include "NorthwestCorner.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>

NorthwestCorner::Result NorthwestCorner::solve(const TransportProblem& problem, bool verbose) {
    Result result;
    result.success = false;
    result.totalCost = 0.0;

    // Проверка валидности
    if (!problem.validate()) {
        result.errorMessage = "Invalid transport problem";
        return result;
    }

    // Проверка баланса
    if (!problem.isBalanced()) {
        result.errorMessage = "Problem must be balanced";
        return result;
    }

    size_t m = problem.m;
    size_t n = problem.n;

    // Инициализация плана нулями
    result.plan.resize(m, std::vector<double>(n, 0.0));

    // Копии запасов и потребностей для работы
    std::vector<double> supply = problem.supply;
    std::vector<double> demand = problem.demand;

    if (verbose) {
        std::cout << "\n=== NORTHWEST CORNER METHOD ===" << std::endl;
        std::cout << "Starting allocation..." << std::endl;
    }

    size_t i = 0, j = 0;
    size_t step = 1;

    // Основной цикл
    while (i < m && j < n) {
        // Выделяем минимум из доступного
        double allocation = std::min(supply[i], demand[j]);
        result.plan[i][j] = allocation;
        result.basis.push_back({i, j});

        if (verbose) {
            printStep(step, i, j, allocation, supply[i] - allocation, demand[j] - allocation, result.plan);
        }

        // Обновляем остатки
        supply[i] -= allocation;
        demand[j] -= allocation;

        // Переходим к следующей клетке
        if (supply[i] < 1e-9 && demand[j] < 1e-9) {
            // Оба исчерпаны (вырожденный случай)
            // Добавляем базисную переменную с нулевым значением
            if (j < n - 1) {
                j++;
            } else if (i < m - 1) {
                i++;
            }
        } else if (supply[i] < 1e-9) {
            // Поставщик исчерпан - идём вниз
            i++;
        } else {
            // Потребитель удовлетворён - идём вправо
            j++;
        }

        step++;
    }

    // Проверка: должно быть m + n - 1 базисных клеток
    size_t expectedBasis = m + n - 1;
    if (result.basis.size() != expectedBasis) {
        // Вырожденный случай - добавляем нулевые базисные переменные
        if (verbose) {
            std::cout << "\nDegenerate case: " << result.basis.size() << " basis cells instead of " << expectedBasis
                      << ". Adding zero allocations..." << std::endl;
        }

        // Добавляем недостающие базисные клетки с epsilon
        const double epsilon = 1e-9;
        for (size_t ii = 0; ii < m && result.basis.size() < expectedBasis; ++ii) {
            for (size_t jj = 0; jj < n && result.basis.size() < expectedBasis; ++jj) {
                bool alreadyBasis = false;
                for (const auto& cell : result.basis) {
                    if (cell.first == ii && cell.second == jj) {
                        alreadyBasis = true;
                        break;
                    }
                }

                if (!alreadyBasis) {
                    result.plan[ii][jj] = epsilon;
                    result.basis.push_back({ii, jj});
                    if (verbose) {
                        std::cout << "Added degenerate basis cell (" << ii << ", " << jj << ") with value " << epsilon
                                  << std::endl;
                    }
                    break;
                }
            }
        }
    }

    // Подсчёт стоимости
    result.totalCost = problem.calculateCost(result.plan);
    result.success = true;

    if (verbose) {
        std::cout << "\nNorthwest Corner Method completed." << std::endl;
        std::cout << "Number of basis cells: " << result.basis.size() << " (expected: " << (m + n - 1) << ")"
                  << std::endl;
        std::cout << "Total cost: " << result.totalCost << std::endl;
        TransportProblem::printPlan(result.plan);
        std::cout << "==============================\n" << std::endl;
    }

    return result;
}

void NorthwestCorner::printStep(size_t step, size_t i, size_t j, double value, double remainingSupply,
                                double remainingDemand, const std::vector<std::vector<double>>& currentPlan) {
    std::cout << "Step " << step << ": Allocate " << std::fixed << std::setprecision(2) << value << " to cell (A"
              << (i + 1) << ", B" << (j + 1) << ")" << std::endl;

    if (remainingSupply < 1e-9 && remainingDemand < 1e-9) {
        std::cout << "  -> Both supply and demand exhausted (degenerate case)" << std::endl;
    } else if (remainingSupply < 1e-9) {
        std::cout << "  -> Supply exhausted, move down" << std::endl;
    } else if (remainingDemand < 1e-9) {
        std::cout << "  -> Demand satisfied, move right" << std::endl;
    }
}
