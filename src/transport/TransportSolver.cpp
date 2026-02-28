#include "TransportSolver.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>

// ==================== TransportSolution ====================

void TransportSolution::print(const std::string& title, const TransportProblem* problem) const {
    std::cout << "\n" << std::string(60, '-') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << std::fixed << std::setprecision(2);

    if (shipments.empty()) {
        std::cout << "  [Нет решения]\n";
        return;
    }

    const size_t m = shipments.size();
    const size_t n = shipments[0].size();

    std::cout << "План перевозок x[i][j]:\n        ";
    for (size_t j = 0; j < n; ++j)
        std::cout << "B" << std::setw(4) << j + 1 << " ";
    std::cout << "\n";

    for (size_t i = 0; i < m; ++i) {
        std::cout << "A" << std::setw(2) << i + 1 << ":   ";
        for (size_t j = 0; j < n; ++j) {
            if (shipments[i][j] > 1e-9) {
                std::cout << std::setw(7) << shipments[i][j] << "*";
            } else {
                std::cout << std::setw(8) << ".";
            }
        }
        std::cout << "\n";
    }

    std::cout << "\n* - базисная переменная\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << "  Стоимость перевозок: " << transportation_cost << "\n";
    if (penalty_cost > 0.0) {
        std::cout << "  Штрафы за недопоставку: " << penalty_cost << "\n";
    }
    std::cout << "  ─────────────────────────────\n";
    std::cout << "  ИТОГО: " << total_cost << "\n";
    std::cout << "  Оптимально: " << (is_optimal ? "ДА" : "НЕТ") << ", итераций: " << iterations << "\n";

    if (problem && problem->hasPenalties()) {
        std::cout << "\n  Детализация по потребителям:\n";
        std::cout << "  Потребитель | Спрос | Получено | Недопоставка | Штраф\n";
        std::cout << "  ------------|-------|----------|--------------|------\n";

        for (size_t j = 0; j < n; ++j) {
            double delivered = getActualDelivery(j);
            const auto& thresholds = problem->penaltyThresholds();
            const auto& rates = problem->penaltyRates();
            const double demand = problem->demands()[j];
            const double threshold = thresholds ? thresholds->at(j) : 0.0;
            const double rate = rates ? rates->at(j) : 0.0;
            const double shortage = std::max(0.0, demand - threshold - delivered);
            const double penalty = shortage * rate;

            std::cout << std::format("  B{:>2d}        | {:>5.0f} | {:>8.0f} | {:>12.0f} | {:>5.2f}\n", j + 1, demand,
                                     delivered, shortage, penalty);
        }
    }

    std::cout << std::string(60, '-') << "\n";
}

// ==================== TransportSolver ====================

void TransportSolver::log(const std::string& message, std::vector<std::string>* log_ptr, bool verbose) {
    if (verbose)
        std::cout << "  [LOG] " << message << "\n";
    if (log_ptr)
        log_ptr->emplace_back(message);
}

TransportSolution TransportSolver::solveNorthwestCorner(const TransportProblem& problem, bool verbose) {
    TransportSolution solution;
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    solution.shipments.assign(m, std::vector<double>(n, 0.0));
    solution.is_basic.assign(m, std::vector<bool>(n, false));

    std::vector<double> avail = problem.supplies();
    std::vector<double> need = problem.demands();

    size_t i = 0, j = 0;
    log("Метод СЗУ: старт с ячейки (1,1)", &solution.log, verbose);

    while (i < m && j < n) {
        const double shipment = std::min(avail[i], need[j]);
        solution.shipments[i][j] = shipment;
        solution.is_basic[i][j] = true;

        log(std::format("  x[{}][{}] = {} (остаток A{}: {}, B{}: {})", i + 1, j + 1, shipment, i + 1,
                        avail[i] - shipment, j + 1, need[j] - shipment),
            &solution.log, verbose);

        const bool supply_exhausted = (avail[i] - shipment < 1e-9);
        const bool demand_exhausted = (need[j] - shipment < 1e-9);

        avail[i] -= shipment;
        need[j] -= shipment;

        if (supply_exhausted && demand_exhausted) {
            if (j + 1 < n) {
                if (!solution.is_basic[i][j + 1]) {
                    solution.shipments[i][j + 1] = 0.0;
                    solution.is_basic[i][j + 1] = true;
                    log(std::format("  [Вырожденность] Добавлена нулевая базисная x[{}][{}] = 0", i + 1, j + 2),
                        &solution.log, verbose);
                }
                ++j;
            } else if (i + 1 < m) {
                if (!solution.is_basic[i + 1][j]) {
                    solution.shipments[i + 1][j] = 0.0;
                    solution.is_basic[i + 1][j] = true;
                    log(std::format("  [Вырожденность] Добавлена нулевая базисная x[{}][{}] = 0", i + 2, j + 1),
                        &solution.log, verbose);
                }
                ++i;
            } else {
                ++i;
                ++j;
            }
        } else if (supply_exhausted) {
            ++i;
        } else if (demand_exhausted) {
            ++j;
        }
    }

    // Гарантируем m+n-1 базисных переменных
    const size_t expected_basis = m + n - 1;
    while (solution.countBasicVars() < expected_basis) {
        for (size_t ii = 0; ii < m && solution.countBasicVars() < expected_basis; ++ii) {
            for (size_t jj = 0; jj < n && solution.countBasicVars() < expected_basis; ++jj) {
                if (!solution.is_basic[ii][jj]) {
                    solution.shipments[ii][jj] = 0.0;
                    solution.is_basic[ii][jj] = true;
                    break;
                }
            }
        }
    }

    // Расчёт стоимости
    solution.transportation_cost = 0.0;
    for (size_t ii = 0; ii < m; ++ii) {
        for (size_t jj = 0; jj < n; ++jj) {
            solution.transportation_cost += problem.costs()[ii][jj] * solution.shipments[ii][jj];
        }
    }

    solution.penalty_cost = problem.calculatePenaltyCost(solution.shipments);
    solution.total_cost = solution.transportation_cost + solution.penalty_cost;

    log(std::format("СЗУ завершено: стоимость = {:.2f}, базисных = {}", solution.total_cost, solution.countBasicVars()),
        &solution.log, verbose);

    return solution;
}

bool TransportSolver::calculatePotentials(const TransportProblem& problem, const TransportSolution& solution,
                                          std::vector<double>& u, std::vector<double>& v, bool verbose) {
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    u.assign(m, 1e20);
    v.assign(n, 1e20);
    u[0] = 0.0;

    bool changed = true;
    int iter = 0;
    const size_t MAX_ITER = m * n * 3;

    while (changed && iter < MAX_ITER) {
        changed = false;
        ++iter;

        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                if (solution.is_basic[i][j]) {
                    const double c = problem.costs()[i][j];

                    if (u[i] < 1e19 && v[j] > 1e19) {
                        v[j] = c - u[i];
                        changed = true;
                    } else if (v[j] < 1e19 && u[i] > 1e19) {
                        u[i] = c - v[j];
                        changed = true;
                    }
                }
            }
        }
    }

    for (size_t i = 0; i < m; ++i)
        if (u[i] > 1e19)
            return false;
    for (size_t j = 0; j < n; ++j)
        if (v[j] > 1e19)
            return false;

    if (verbose) {
        std::cout << "  Потенциалы u: ";
        for (double val : u)
            std::cout << std::fixed << std::setprecision(2) << val << " ";
        std::cout << "\n  Потенциалы v: ";
        for (double val : v)
            std::cout << std::fixed << std::setprecision(2) << val << " ";
        std::cout << "\n";
    }

    return true;
}

bool TransportSolver::findImprovingCell(const TransportProblem& problem, const std::vector<double>& u,
                                        const std::vector<double>& v, const TransportSolution& solution, size_t& out_i,
                                        size_t& out_j, double& out_delta, bool verbose) {
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    out_delta = 0.0;
    bool found = false;

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (!solution.is_basic[i][j]) {
                const double delta = problem.costs()[i][j] - u[i] - v[j];

                if (delta < -1e-9 && (!found || delta < out_delta)) {
                    out_i = i;
                    out_j = j;
                    out_delta = delta;
                    found = true;
                }
            }
        }
    }

    if (found && verbose) {
        std::cout << std::fixed << std::setprecision(2);
        log(std::format("Найдена улучшающая ячейка ({},{}): Δ = {:.3f}", out_i + 1, out_j + 1, out_delta), nullptr,
            true);
    }

    return found;
}

bool TransportSolver::findCycle(const TransportSolution& solution, size_t start_i, size_t start_j,
                                std::vector<std::pair<size_t, size_t>>& cycle) {
    const size_t m = solution.shipments.size();
    const size_t n = solution.shipments[0].size();

    cycle.clear();
    cycle.emplace_back(start_i, start_j);

    // Простой поиск цикла через чередование строка/столбец
    size_t cur_i = start_i, cur_j = start_j;
    bool move_horizontal = true;
    std::vector<std::vector<bool>> visited(m, std::vector<bool>(n, false));

    for (int step = 0; step < m + n + 2; ++step) {
        bool found = false;

        if (move_horizontal) {
            for (size_t jj = 0; jj < n; ++jj) {
                if (jj != cur_j && solution.is_basic[cur_i][jj]) {
                    if (cur_i == start_i && step >= 2) {
                        // Можем замкнуть
                        for (size_t ii = 0; ii < m; ++ii) {
                            if (ii != cur_i && solution.is_basic[ii][cur_j]) {
                                cycle.emplace_back(cur_i, jj);
                                cycle.emplace_back(ii, jj);
                                cycle.emplace_back(ii, cur_j);
                                cycle.emplace_back(start_i, start_j);
                                return cycle.size() >= 5;
                            }
                        }
                    }
                    if (!visited[cur_i][jj]) {
                        visited[cur_i][jj] = true;
                        cycle.emplace_back(cur_i, jj);
                        cur_j = jj;
                        found = true;
                        break;
                    }
                }
            }
        } else {
            for (size_t ii = 0; ii < m; ++ii) {
                if (ii != cur_i && solution.is_basic[ii][cur_j]) {
                    if (cur_j == start_j && step >= 2) {
                        for (size_t jj = 0; jj < n; ++jj) {
                            if (jj != cur_j && solution.is_basic[cur_i][jj]) {
                                cycle.emplace_back(ii, cur_j);
                                cycle.emplace_back(ii, jj);
                                cycle.emplace_back(cur_i, jj);
                                cycle.emplace_back(start_i, start_j);
                                return cycle.size() >= 5;
                            }
                        }
                    }
                    if (!visited[ii][cur_j]) {
                        visited[ii][cur_j] = true;
                        cycle.emplace_back(ii, cur_j);
                        cur_i = ii;
                        found = true;
                        break;
                    }
                }
            }
        }

        if (!found) {
            if (cycle.size() <= 1)
                return false;
            cycle.pop_back();
            if (!cycle.empty()) {
                cur_i = cycle.back().first;
                cur_j = cycle.back().second;
            }
            move_horizontal = !move_horizontal;
        } else {
            move_horizontal = !move_horizontal;
        }
    }

    return false;
}

void TransportSolver::redistributeAlongCycle(TransportSolution& solution,
                                             const std::vector<std::pair<size_t, size_t>>& cycle, double theta) {
    if (cycle.size() < 4)
        return;

    bool add = true;
    for (size_t k = 0; k < cycle.size() - 1; ++k) {
        const auto& [i, j] = cycle[k];

        if (add) {
            solution.shipments[i][j] += theta;
        } else {
            solution.shipments[i][j] -= theta;
            if (solution.shipments[i][j] < 0)
                solution.shipments[i][j] = 0;
        }
        add = !add;
    }

    // Обновляем is_basic
    for (size_t i = 0; i < solution.shipments.size(); ++i) {
        for (size_t j = 0; j < solution.shipments[i].size(); ++j) {
            if (solution.shipments[i][j] > 1e-9) {
                solution.is_basic[i][j] = true;
            }
        }
    }
}

TransportSolution TransportSolver::solvePotentials(const TransportProblem& problem,
                                                   const TransportSolution& initial_solution, bool verbose) {
    TransportSolution solution = initial_solution;
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    log("=== Метод потенциалов: начало оптимизации ===", &solution.log, verbose);

    while (true) {
        ++solution.iterations;
        if (verbose) {
            std::cout << "\n[Итерация " << solution.iterations << "]\n";
            std::cout << "  Базисных переменных: " << solution.countBasicVars() << " (ожидалось " << m + n - 1 << ")\n";
        }

        std::vector<double> u, v;
        if (!calculatePotentials(problem, solution, u, v, verbose)) {
            log("Ошибка вычисления потенциалов", &solution.log, verbose);
            break;
        }

        size_t imp_i, imp_j;
        double delta;
        if (!findImprovingCell(problem, u, v, solution, imp_i, imp_j, delta, verbose)) {
            log("Улучшающих клеток не найдено -> решение оптимально", &solution.log, verbose);
            solution.is_optimal = true;
            break;
        }

        std::vector<std::pair<size_t, size_t>> cycle;
        if (!findCycle(solution, imp_i, imp_j, cycle)) {
            log("Не удалось найти цикл", &solution.log, verbose);
            break;
        }

        if (verbose) {
            std::cout << "  Цикл: ";
            for (const auto& [i, j] : cycle) {
                std::cout << "(" << i + 1 << "," << j + 1 << ") ";
            }
            std::cout << "\n";
        }

        double theta = std::numeric_limits<double>::max();
        bool minus = false;

        for (size_t k = 1; k < cycle.size() - 1; ++k) {
            const auto& [i, j] = cycle[k];
            if (minus) {
                theta = std::min(theta, solution.shipments[i][j]);
            }
            minus = !minus;
        }

        if (theta > std::numeric_limits<double>::max() / 2) {
            theta = 0.0;
        }

        log(std::format("  Theta = {:.3f}", theta), &solution.log, verbose);

        redistributeAlongCycle(solution, cycle, theta);

        // Пересчёт стоимости
        solution.transportation_cost = 0.0;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                solution.transportation_cost += problem.costs()[i][j] * solution.shipments[i][j];
            }
        }
        solution.penalty_cost = problem.calculatePenaltyCost(solution.shipments);
        solution.total_cost = solution.transportation_cost + solution.penalty_cost;

        log(std::format("  Новая стоимость: {:.2f}", solution.total_cost), &solution.log, verbose);

        if (solution.iterations > m * n * 10) {
            log("Превышено максимальное число итераций", &solution.log, verbose);
            break;
        }
    }

    log(std::format("Метод потенциалов завершён: итераций = {}, оптимально = {}", solution.iterations,
                    solution.is_optimal),
        &solution.log, verbose);

    return solution;
}

TransportSolution TransportSolver::solve(const TransportProblem& problem, bool verbose) {
    if (verbose) {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "  РЕШЕНИЕ ТРАНСПОРТНОЙ ЗАДАЧИ\n";
        std::cout << std::string(60, '=') << "\n";
    }

    TransportProblem balanced = problem.balance();
    if (!problem.isBalanced() && verbose) {
        std::cout << "[INFO] Задача сбалансирована добавлением фиктивного узла\n";
    }

    TransportSolution initial = solveNorthwestCorner(balanced, verbose);
    TransportSolution optimal = solvePotentials(balanced, initial, verbose);

    if (verbose) {
        optimal.print("ФИНАЛЬНОЕ РЕШЕНИЕ", &problem);
    }

    return optimal;
}
