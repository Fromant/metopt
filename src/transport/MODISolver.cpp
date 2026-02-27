#include "MODISolver.hpp"

#include <functional>
#include <iomanip>
#include <iostream>
#include <queue>
#include <set>

MODISolver::Result MODISolver::solve(const TransportProblem& problem,
                                     const std::vector<std::vector<double>>& initialPlan,
                                     const std::vector<std::pair<size_t, size_t>>& initialBasis, bool verbose,
                                     int maxIterations) {
    Result result;
    result.success = false;
    result.iterations = 0;
    result.isOptimal = false;
    result.totalCost = 0;

    // Проверки
    if (!problem.validate()) {
        result.errorMessage = "Invalid transport problem";
        return result;
    }

    if (!problem.isBalanced()) {
        result.errorMessage = "Problem must be balanced";
        return result;
    }

    if (initialPlan.empty() || initialBasis.empty()) {
        result.errorMessage = "Initial plan and basis required";
        return result;
    }

    // Копируем начальный план
    result.plan = initialPlan;
    result.basis = initialBasis;

    if (verbose) {
        std::cout << "\n=== MODI METHOD (POTENTIALS) ===" << std::endl;
        std::cout << "Initial cost: " << problem.calculateCost(result.plan) << std::endl;
        std::cout << "Initial basis size: " << result.basis.size() << std::endl;
    }

    // Основной цикл
    while (result.iterations < maxIterations) {
        result.iterations++;

        if (verbose) {
            std::cout << "\n--- Iteration " << result.iterations << " ---" << std::endl;
        }

        // Шаг 1: Вычисляем потенциалы
        std::vector<double> u(problem.m, 0.0);
        std::vector<double> v(problem.n, 0.0);

        if (!computePotentials(problem, result.basis, u, v)) {
            result.errorMessage = "Failed to compute potentials";
            return result;
        }

        if (verbose) {
            printPotentials(u, v);
        }

        // Шаг 2: Вычисляем оценки свободных клеток
        if (verbose) {
            printDeltas(problem, result.basis, u, v);
        }

        // Шаг 3: Ищем клетку для входа в базис
        auto enteringCell = findEnteringCell(problem, result.basis, u, v);

        if (!enteringCell.has_value()) {
            // Все оценки >= 0, план оптимален
            result.isOptimal = true;
            result.success = true;
            result.totalCost = problem.calculateCost(result.plan);

            if (verbose) {
                std::cout << "\nOptimal solution found!" << std::endl;
                std::cout << "Total iterations: " << result.iterations << std::endl;
                std::cout << "Optimal cost: " << result.totalCost << std::endl;
                TransportProblem::printPlan(result.plan);
                std::cout << "==============================\n" << std::endl;
            }

            return result;
        }

        size_t enterI = enteringCell->first;
        size_t enterJ = enteringCell->second;
        double delta = computeDelta(problem, enterI, enterJ, u, v);

        if (verbose) {
            std::cout << "\nEntering cell: (A" << (enterI + 1) << ", B" << (enterJ + 1) << ") with delta = " << delta
                      << std::endl;
        }

        // Шаг 4: Строим цикл
        std::vector<std::pair<size_t, size_t>> cycle = findCycle(enterI, enterJ, result.basis);

        if (cycle.empty()) {
            result.errorMessage = "Failed to find cycle";
            return result;
        }

        if (verbose) {
            printCycle(cycle, result.plan);
        }

        // Шаг 5: Применяем цикл
        size_t leavingIndex;
        applyCycle(result.plan, cycle, leavingIndex);

        // Обновляем базис
        result.basis.emplace_back(enterI, enterJ);
        if (leavingIndex < result.basis.size() - 1) {
            result.basis.erase(result.basis.begin() + leavingIndex);
        }

        if (verbose) {
            std::cout << "New cost: " << problem.calculateCost(result.plan) << std::endl;
        }
    }

    result.errorMessage = "Maximum iterations reached";
    result.totalCost = problem.calculateCost(result.plan);
    return result;
}

bool MODISolver::computePotentials(const TransportProblem& problem, const std::vector<std::pair<size_t, size_t>>& basis,
                                   std::vector<double>& u, std::vector<double>& v) {
    // u[0] = 0
    u[0] = 0.0;

    // Система уравнений: u[i] + v[j] = cost[i][j] для базисных клеток
    // Используем простой итерационный метод

    bool changed = true;
    int maxIter = 1000;
    int iter = 0;

    while (changed && iter < maxIter) {
        changed = false;
        iter++;

        for (const auto& cell : basis) {
            size_t i = cell.first;
            size_t j = cell.second;
            double c = problem.cost[i][j];

            // Если u[i] известно, вычисляем v[j]
            if (std::abs(u[i]) < 1e10 && std::abs(v[j]) > 1e10) {
                v[j] = c - u[i];
                changed = true;
            }
            // Если v[j] известно, вычисляем u[i]
            else if (std::abs(v[j]) < 1e10 && std::abs(u[i]) > 1e10) {
                u[i] = c - v[j];
                changed = true;
            }
        }
    }

    // Инициализируем неизвестные нулями
    for (size_t i = 0; i < problem.m; ++i) {
        if (std::abs(u[i]) > 1e10)
            u[i] = 0.0;
    }
    for (size_t j = 0; j < problem.n; ++j) {
        if (std::abs(v[j]) > 1e10)
            v[j] = 0.0;
    }

    return iter < maxIter;
}

double MODISolver::computeDelta(const TransportProblem& problem, size_t i, size_t j, const std::vector<double>& u,
                                const std::vector<double>& v) {
    return problem.cost[i][j] - u[i] - v[j];
}

std::optional<std::pair<size_t, size_t>>
MODISolver::findEnteringCell(const TransportProblem& problem, const std::vector<std::pair<size_t, size_t>>& basis,
                             const std::vector<double>& u, const std::vector<double>& v) {

    std::optional<std::pair<size_t, size_t>> bestCell;
    double minDelta = 0.0;

    // Создаём множество базисных клеток для быстрой проверки
    std::set<std::pair<size_t, size_t>> basisSet(basis.begin(), basis.end());

    for (size_t i = 0; i < problem.m; ++i) {
        for (size_t j = 0; j < problem.n; ++j) {
            // Пропускаем базисные клетки
            if (basisSet.contains({i, j})) {
                continue;
            }

            double delta = computeDelta(problem, i, j, u, v);

            if (delta < minDelta - 1e-9) {
                minDelta = delta;
                bestCell = {i, j};
            }
        }
    }

    return bestCell;
}

std::vector<std::pair<size_t, size_t>> MODISolver::findCycle(size_t startI, size_t startJ,
                                                             const std::vector<std::pair<size_t, size_t>>& basis) {

    // Используем DFS для поиска цикла
    // Цикл должен начинаться и заканчиваться в (startI, startJ)
    // и проходить только через базисные клетки (кроме начальной)

    std::vector<std::pair<size_t, size_t>> cycle;
    cycle.emplace_back(startI, startJ);

    // Создаём граф смежности для базисных клеток
    std::set<std::pair<size_t, size_t>> basisSet(basis.begin(), basis.end());

    // Функция для поиска пути
    std::function<bool(size_t, size_t, std::vector<std::pair<size_t, size_t>>&, std::set<std::pair<size_t, size_t>>)>
        dfs;

    dfs = [&](size_t i, size_t j, std::vector<std::pair<size_t, size_t>>& path,
              std::set<std::pair<size_t, size_t>> visited) -> bool {
        // Пробуем двигаться по строке i
        for (size_t jj = 0; jj < 100; ++jj) { // Ограничение для безопасности
            if (jj == j)
                continue;

            auto cell = std::make_pair(i, jj);
            if (visited.contains(cell))
                continue;

            if (basisSet.contains(cell) || cell == std::make_pair(startI, startJ)) {
                path.emplace_back(cell);
                visited.insert(cell);

                if (cell == std::make_pair(startI, startJ) && path.size() > 1) {
                    return true; // Нашли цикл
                }

                if (dfs(i, jj, path, visited)) {
                    return true;
                }

                path.pop_back();
            }
        }

        // Пробуем двигаться по столбцу j
        for (size_t ii = 0; ii < 100; ++ii) {
            if (ii == i)
                continue;

            auto cell = std::make_pair(ii, j);
            if (visited.contains(cell))
                continue;

            if (basisSet.contains(cell) || cell == std::make_pair(startI, startJ)) {
                path.emplace_back(cell);
                visited.insert(cell);

                if (cell == std::make_pair(startI, startJ) && path.size() > 1) {
                    return true;
                }

                if (dfs(ii, j, path, visited)) {
                    return true;
                }

                path.pop_back();
            }
        }

        return false;
    };

    std::set<std::pair<size_t, size_t>> visited;
    visited.insert({startI, startJ});

    if (dfs(startI, startJ, cycle, visited)) {
        return cycle;
    }

    return {};
}

void MODISolver::applyCycle(std::vector<std::vector<double>>& plan, const std::vector<std::pair<size_t, size_t>>& cycle,
                            size_t& leavingIndex) {

    if (cycle.size() < 2) {
        leavingIndex = 0;
        return;
    }

    // Находим минимальное значение в клетках со знаком "-"
    double theta = 1e10;
    for (size_t k = 1; k < cycle.size(); k += 2) { // Нечётные индексы - знак "-"
        auto [i, j] = cycle[k];
        if (plan[i][j] < theta) {
            theta = plan[i][j];
            leavingIndex = k;
        }
    }

    // Применяем сдвиг
    for (size_t k = 0; k < cycle.size(); ++k) {
        auto [i, j] = cycle[k];
        if (k % 2 == 0) {
            plan[i][j] += theta; // Знак "+"
        } else {
            plan[i][j] -= theta; // Знак "-"
            if (plan[i][j] < 1e-9) {
                plan[i][j] = 0.0;
            }
        }
    }
}

void MODISolver::printPotentials(const std::vector<double>& u, const std::vector<double>& v) {
    std::cout << "Potentials:" << std::endl;
    std::cout << "  u = [";
    for (size_t i = 0; i < u.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2) << u[i];
        if (i < u.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "  v = [";
    for (size_t j = 0; j < v.size(); ++j) {
        std::cout << std::fixed << std::setprecision(2) << v[j];
        if (j < v.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void MODISolver::printDeltas(const TransportProblem& problem, const std::vector<std::pair<size_t, size_t>>& basis,
                             const std::vector<double>& u, const std::vector<double>& v) {
    std::set<std::pair<size_t, size_t>> basisSet(basis.begin(), basis.end());

    std::cout << "Delta matrix (free cells only):" << std::endl;
    std::cout << "       ";
    for (size_t j = 0; j < problem.n; ++j) {
        std::cout << std::setw(10) << ("B" + std::to_string(j + 1));
    }
    std::cout << std::endl;

    for (size_t i = 0; i < problem.m; ++i) {
        std::cout << "A" << (i + 1) << "  ";
        for (size_t j = 0; j < problem.n; ++j) {
            if (basisSet.contains({i, j})) {
                std::cout << std::setw(10) << "-";
            } else {
                double delta = computeDelta(problem, i, j, u, v);
                std::cout << std::setw(10) << std::fixed << std::setprecision(2) << delta;
            }
        }
        std::cout << std::endl;
    }
}

void MODISolver::printCycle(const std::vector<std::pair<size_t, size_t>>& cycle,
                            const std::vector<std::vector<double>>& plan) {
    std::cout << "Cycle: ";
    for (size_t k = 0; k < cycle.size(); ++k) {
        auto [i, j] = cycle[k];
        std::cout << "(A" << (i + 1) << ",B" << (j + 1) << ")";
        if (k % 2 == 0) {
            std::cout << "[+]";
        } else {
            std::cout << "[-](" << std::fixed << std::setprecision(2) << plan[i][j] << ")";
        }
        if (k < cycle.size() - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;
}
