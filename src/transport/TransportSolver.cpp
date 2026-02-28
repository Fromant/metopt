#include "transport/TransportSolver.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>

// ==================== TransportSolution ====================

void TransportSolution::print(const std::string& title,
                             const TransportProblem* problem) const {
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
    for (size_t j = 0; j < n; ++j) std::cout << "B" << std::setw(4) << j+1 << " ";
    std::cout << "\n";

    for (size_t i = 0; i < m; ++i) {
        std::cout << "A" << std::setw(2) << i+1 << ":   ";
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
    std::cout << "  Оптимально: " << (is_optimal ? "ДА" : "НЕТ")
              << ", итераций: " << iterations << "\n";

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

            std::cout << std::format("  B{:>2d}        | {:>5.0f} | {:>8.0f} | {:>12.0f} | {:>5.2f}\n",
                j+1, demand, delivered, shortage, penalty);
        }
    }

    std::cout << std::string(60, '-') << "\n";
}

// ==================== TransportSolver ====================

void TransportSolver::log(const std::string& message,
                         std::vector<std::string>* log_ptr,
                         bool verbose) {
    if (verbose) std::cout << "  [LOG] " << message << "\n";
    if (log_ptr) log_ptr->push_back(message);
}

// 🎯 Создаём расширенную задачу
static TransportProblem createExpandedProblem(const TransportProblem& original) {
    if (!original.hasPenalties()) {
        return original.balance();
    }

    const size_t m = original.numSuppliers();
    const size_t n = original.numConsumers();

    const size_t m_expanded = m + 1;
    const size_t n_expanded = n + 1;

    std::vector<double> supplies(m_expanded);
    std::vector<double> demands(n_expanded);
    std::vector<std::vector<double>> costs(m_expanded, std::vector<double>(n_expanded, 0.0));

    for (size_t i = 0; i < m; ++i) {
        supplies[i] = original.supplies()[i];
    }

    double dummy_supply = 0.0;
    for (size_t j = 0; j < n; ++j) {
        double max_shortage = original.demands()[j] - original.penaltyThresholds()->at(j);
        dummy_supply += std::max(0.0, max_shortage);
    }
    supplies[m] = dummy_supply;

    for (size_t j = 0; j < n; ++j) {
        demands[j] = original.demands()[j];
    }

    demands[n] = original.totalSupply() + dummy_supply - original.totalDemand();

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            costs[i][j] = original.costs()[i][j];
        }
        costs[i][n] = 0.0;
    }

    for (size_t j = 0; j < n; ++j) {
        costs[m][j] = original.penaltyRates()->at(j);
    }
    costs[m][n] = 0.0;

    TransportProblem expanded(supplies, demands, costs);
    return expanded;
}

static std::vector<std::vector<double>> extractOriginalPlan(
    const std::vector<std::vector<double>>& expanded_plan,
    size_t original_m, size_t original_n)
{
    std::vector<std::vector<double>> original_plan(original_m, std::vector<double>(original_n, 0.0));
    for (size_t i = 0; i < original_m; ++i) {
        for (size_t j = 0; j < original_n; ++j) {
            original_plan[i][j] = expanded_plan[i][j];
        }
    }
    return original_plan;
}

// 🎯 СТАРАЯ РАБОЧАЯ ВЕРСИЯ СЗУ (с разделением на реальных и фиктивных)
TransportSolution TransportSolver::solveNorthwestCorner(
    const TransportProblem& problem, bool verbose)
{
    TransportSolution solution;
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    solution.shipments.assign(m, std::vector<double>(n, 0.0));
    solution.is_basic.assign(m, std::vector<bool>(n, false));

    std::vector<double> avail = problem.supplies();
    std::vector<double> need = problem.demands();

    // Определяем наличие фиктивных строки/столбца по стоимости
    const bool has_dummy_supplier = (m > 1 && problem.costs()[m-1][0] < 100);
    const bool has_dummy_consumer = (n > 1 && problem.costs()[0][n-1] < 1e-6);
    const size_t real_m = has_dummy_supplier ? m - 1 : m;
    const size_t real_n = has_dummy_consumer ? n - 1 : n;

    log(std::format("Метод СЗУ: {} реальных поставщиков, {} реальных потребителей",
                   real_m, real_n), &solution.log, verbose);

    // ШАГ 1: Реальные поставщики → реальные потребители
    size_t i = 0, j = 0;
    while (i < real_m && j < real_n) {
        double shipment = std::min(avail[i], need[j]);
        solution.shipments[i][j] = shipment;
        solution.is_basic[i][j] = true;

        log(std::format("  x[{}][{}] = {} (остаток A{}: {:.2f}, B{}: {:.2f})",
                       i+1, j+1, shipment, i+1, avail[i]-shipment, j+1, need[j]-shipment),
            &solution.log, verbose);

        avail[i] -= shipment;
        need[j] -= shipment;

        // Если и запас, и спрос обнулились, добавляем нулевую базисную в следующую клетку
        if (avail[i] < 1e-9 && need[j] < 1e-9) {
            if (j + 1 < real_n) {
                if (!solution.is_basic[i][j+1]) {
                    solution.shipments[i][j+1] = 0.0;
                    solution.is_basic[i][j+1] = true;
                }
                ++j;
            } else if (i + 1 < real_m) {
                ++i;
            } else {
                ++i; ++j;
            }
        } else if (avail[i] < 1e-9) {
            ++i;
        } else {
            ++j;
        }
    }

    // ШАГ 2: Остатки реальных поставщиков → фиктивному потребителю
    if (has_dummy_consumer) {
        for (i = 0; i < real_m; ++i) {
            if (avail[i] > 1e-9) {
                solution.shipments[i][real_n] = avail[i];
                solution.is_basic[i][real_n] = true;
                log(std::format("  x[{}][{}] = {} (фиктивный потребитель)",
                               i+1, real_n+1, avail[i]), &solution.log, verbose);
                avail[i] = 0;
            }
        }
    }

    // ШАГ 3: Фиктивный поставщик → оставшийся спрос реальных потребителей
    if (has_dummy_supplier) {
        const size_t dummy_i = real_m;
        for (j = 0; j < real_n; ++j) {
            if (need[j] > 1e-9 && avail[dummy_i] > 1e-9) {
                double shipment = std::min(avail[dummy_i], need[j]);
                solution.shipments[dummy_i][j] = shipment;
                solution.is_basic[dummy_i][j] = true;
                log(std::format("  x[{}][{}] = {} (ФИКТИВНЫЙ = недопоставка)",
                               dummy_i+1, j+1, shipment), &solution.log, verbose);
                avail[dummy_i] -= shipment;
                need[j] -= shipment;
            }
        }
        // Остаток фиктивного поставщика → фиктивному потребителю
        if (avail[dummy_i] > 1e-9 && has_dummy_consumer) {
            solution.shipments[dummy_i][real_n] = avail[dummy_i];
            solution.is_basic[dummy_i][real_n] = true;
        }
    }

    // ШАГ 4: ГАРАНТИРУЕМ СВЯЗНОСТЬ – добавляем вырожденную связь фиктивного поставщика с реальным потребителем
    if (has_dummy_supplier) {
        const size_t dummy_i = real_m;
        bool connected_to_real = false;
        for (j = 0; j < real_n; ++j) {
            if (solution.is_basic[dummy_i][j]) {
                connected_to_real = true;
                break;
            }
        }
        if (!connected_to_real) {
            size_t best_j = 0;
            double min_penalty = problem.costs()[dummy_i][0];
            for (j = 1; j < real_n; ++j) {
                if (problem.costs()[dummy_i][j] < min_penalty) {
                    min_penalty = problem.costs()[dummy_i][j];
                    best_j = j;
                }
            }
            solution.shipments[dummy_i][best_j] = 0.0;
            solution.is_basic[dummy_i][best_j] = true;
            log(std::format("  [СВЯЗНОСТЬ] Добавлена вырожденная x[{}][{}] = 0 (penalty={:.2f})",
                           dummy_i+1, best_j+1, min_penalty), &solution.log, verbose);
        }
    }

    // ШАГ 5: ДОВОДИМ КОЛИЧЕСТВО БАЗИСНЫХ ДО m+n-1
    const size_t expected_basis = m + n - 1;
    while (solution.countBasicVars() < expected_basis) {
        for (size_t ii = 0; ii < m && solution.countBasicVars() < expected_basis; ++ii) {
            for (size_t jj = 0; jj < n && solution.countBasicVars() < expected_basis; ++jj) {
                if (!solution.is_basic[ii][jj]) {
                    solution.shipments[ii][jj] = 0.0;
                    solution.is_basic[ii][jj] = true;
                    log(std::format("  [Вырожденность] Добавлена нулевая базисная x[{}][{}] = 0",
                                   ii+1, jj+1), &solution.log, verbose);
                    break;
                }
            }
        }
    }

    log(std::format("СЗУ завершено: базисных = {} (ожидалось {})",
                   solution.countBasicVars(), expected_basis),
        &solution.log, verbose);

    return solution;
}

bool TransportSolver::calculatePotentials(const TransportProblem& problem,
                                         const TransportSolution& solution,
                                         std::vector<double>& u,
                                         std::vector<double>& v,
                                         bool verbose)
{
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    u.assign(m, 1e20);
    v.assign(n, 1e20);
    u[0] = 0.0;

    if (verbose) {
        std::cout << "  Базисные клетки для потенциалов:\n";
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                if (solution.is_basic[i][j]) {
                    std::cout << "    (" << i+1 << "," << j+1 << "): c="
                              << problem.costs()[i][j] << "\n";
                }
            }
        }
    }

    bool changed = true;
    int iter = 0;
    const int MAX_ITER = m * n * 5;

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
                    }
                    else if (v[j] < 1e19 && u[i] > 1e19) {
                        u[i] = c - v[j];
                        changed = true;
                    }
                }
            }
        }
    }

    bool all_computed = true;
    for (size_t i = 0; i < m; ++i)
        if (u[i] > 1e19) {
            if (verbose) std::cout << "  [WARN] Не вычислен u[" << i << "]\n";
            all_computed = false;
        }
    for (size_t j = 0; j < n; ++j)
        if (v[j] > 1e19) {
            if (verbose) std::cout << "  [WARN] Не вычислен v[" << j << "]\n";
            all_computed = false;
        }

    if (!all_computed) {
        if (verbose) std::cout << "  [ERROR] Базис несвязный!\n";
        return false;
    }

    if (verbose) {
        std::cout << "  Потенциалы u: ";
        for (double val : u) std::cout << std::fixed << std::setprecision(2) << val << " ";
        std::cout << "\n  Потенциалы v: ";
        for (double val : v) std::cout << std::fixed << std::setprecision(2) << val << " ";
        std::cout << "\n";
    }

    return true;
}

bool TransportSolver::findImprovingCell(const TransportProblem& problem,
                                       const std::vector<double>& u,
                                       const std::vector<double>& v,
                                       const TransportSolution& solution,
                                       size_t& out_i, size_t& out_j,
                                       double& out_delta,
                                       bool& out_is_penalty,
                                       bool verbose)
{
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    out_delta = 0.0;
    out_is_penalty = false;
    bool found = false;

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (!solution.is_basic[i][j]) {
                const double delta = problem.costs()[i][j] - u[i] - v[j];

                if (delta < -1e-9 && (!found || delta < out_delta)) {
                    out_i = i;
                    out_j = j;
                    out_delta = delta;
                    out_is_penalty = (i == m - 1);
                    found = true;
                }
            }
        }
    }

    if (found && verbose) {
        if (out_is_penalty) {
            log(std::format("Найдена улучшающая ячейка штрафа (A{},B{}): Δ = {:.3f}",
                           out_i+1, out_j+1, out_delta), nullptr, verbose);
        } else {
            log(std::format("Найдена улучшающая ячейка ({},{}): Δ = {:.3f}",
                           out_i+1, out_j+1, out_delta), nullptr, verbose);
        }
    }

    return found;
}

// 🎯 ИСПРАВЛЕННЫЙ ПОИСК ЦИКЛА (гарантирует правильный порядок: start первый, start последний)
bool TransportSolver::findCycle(const TransportSolution& solution,
                               size_t start_i, size_t start_j,
                               std::vector<std::pair<size_t, size_t>>& cycle,
                               bool /*is_penalty_entering*/)
{
    const size_t m = solution.shipments.size();
    const size_t n = solution.shipments[0].size();

    // Строим списки базисных клеток (без стартовой)
    std::vector<std::vector<size_t>> basic_in_row(m);
    std::vector<std::vector<size_t>> basic_in_col(n);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            if (solution.is_basic[i][j]) {
                basic_in_row[i].push_back(j);
                basic_in_col[j].push_back(i);
            }
        }
    }

    // Добавляем стартовую клетку в списки (она временно станет "базисной" для поиска)
    basic_in_row[start_i].push_back(start_j);
    basic_in_col[start_j].push_back(start_i);

    std::vector<std::pair<size_t, size_t>> path; // путь от первого шага до предпоследнего
    std::vector<std::vector<bool>> visited(m, std::vector<bool>(n, false));

    // Рекурсивный поиск: начинаем с соседей start, не включая саму start
    std::function<bool(size_t, size_t, bool)> dfs =
        [&](size_t i, size_t j, bool last_horizontal) -> bool
        {
            // Если вернулись в стартовую клетку
            if (i == start_i && j == start_j) {
                if (path.size() >= 3) { // минимум 3 промежуточных клетки
                    cycle.clear();
                    // Правильный порядок: start, затем path, затем start
                    cycle.push_back({start_i, start_j});
                    for (const auto& p : path) cycle.push_back(p);
                    cycle.push_back({start_i, start_j});
                    return true;
                }
                return false;
            }

            if (visited[i][j]) return false;

            visited[i][j] = true;
            path.push_back({i, j});

            if (!last_horizontal) {
                // Горизонтальное движение
                for (size_t jj : basic_in_row[i]) {
                    if (jj == j) continue;
                    if (dfs(i, jj, true)) return true;
                }
            } else {
                // Вертикальное движение
                for (size_t ii : basic_in_col[j]) {
                    if (ii == i) continue;
                    if (dfs(ii, j, false)) return true;
                }
            }

            path.pop_back();
            visited[i][j] = false;
            return false;
        };

    // Пробуем начать с горизонтального движения из стартовой клетки
    for (size_t jj : basic_in_row[start_i]) {
        if (jj == start_j) continue;
        if (dfs(start_i, jj, true)) return true;
    }
    // Пробуем начать с вертикального движения
    for (size_t ii : basic_in_col[start_j]) {
        if (ii == start_i) continue;
        if (dfs(ii, start_j, false)) return true;
    }

    return false;
}

// 🎯 НОВАЯ РЕАЛИЗАЦИЯ ПЕРЕРАСПРЕДЕЛЕНИЯ ПО ЦИКЛУ (возвращает θ и выходящую клетку)
double TransportSolver::redistributeAlongCycle(
    TransportSolution& solution,
    const std::vector<std::pair<size_t, size_t>>& cycle,
    std::pair<size_t, size_t>& exiting_cell,
    bool verbose)
{
    if (cycle.size() < 4) return 0.0;

    // Определяем знаки: первая клетка (вводимая) – плюс, далее чередуем
    double theta = std::numeric_limits<double>::max();
    // Минусовые клетки имеют нечётные индексы в цикле (начиная с 1)
    for (size_t k = 1; k < cycle.size() - 1; ++k) {
        if (k % 2 == 1) { // нечётные индексы — минусовые
            const auto& [i, j] = cycle[k];
            theta = std::min(theta, solution.shipments[i][j]);
        }
    }

    if (theta > std::numeric_limits<double>::max() / 2) {
        theta = 0.0;
    }

    // Перераспределяем поставки по циклу (идём до предпоследней клетки)
    bool add = true;
    for (size_t k = 0; k < cycle.size() - 1; ++k) {
        const auto& [i, j] = cycle[k];
        if (add) {
            solution.shipments[i][j] += theta;
        } else {
            solution.shipments[i][j] -= theta;
            if (solution.shipments[i][j] < 0) solution.shipments[i][j] = 0; // защита
        }
        add = !add;
    }

    // Ищем выходящую клетку (среди минусовых, где значение стало равно 0)
    exiting_cell = {std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()};
    for (size_t k = 1; k < cycle.size() - 1; ++k) {
        if (k % 2 == 1) {
            const auto& [i, j] = cycle[k];
            if (solution.shipments[i][j] <= 1e-9) {
                exiting_cell = {i, j};
                break;
            }
        }
    }

    // Если не нашли (например, при θ=0 все минусовые могут быть нулевыми), берём первую минусовую
    if (exiting_cell.first == std::numeric_limits<size_t>::max()) {
        for (size_t k = 1; k < cycle.size() - 1; ++k) {
            if (k % 2 == 1) {
                const auto& [i, j] = cycle[k];
                exiting_cell = {i, j};
                break;
            }
        }
    }

    if (verbose) {
        std::cout << "  Theta = " << std::fixed << std::setprecision(3) << theta
                  << ", exiting cell: (" << exiting_cell.first+1 << ","
                  << exiting_cell.second+1 << ")\n";
    }

    return theta;
}

// 🎯 ИСПРАВЛЕННЫЙ МЕТОД ПОТЕНЦИАЛОВ (с использованием новых функций)
TransportSolution TransportSolver::solvePotentials(
    const TransportProblem& problem,
    const TransportSolution& initial_solution,
    bool verbose)
{
    TransportSolution solution = initial_solution;
    const size_t m = problem.numSuppliers();
    const size_t n = problem.numConsumers();

    log("=== Метод потенциалов: начало оптимизации ===", &solution.log, verbose);

    const size_t MAX_TOTAL_ITER = (m + n) * 50;
    int degenerate_count = 0;
    double prev_cost = solution.total_cost;
    int no_improvement_count = 0;

    while (solution.iterations < MAX_TOTAL_ITER) {
        ++solution.iterations;
        if (verbose) {
            std::cout << "\n[Итерация " << solution.iterations << "]\n";
        }

        std::vector<double> u, v;
        if (!calculatePotentials(problem, solution, u, v, verbose)) {
            log("Ошибка вычисления потенциалов", &solution.log, verbose);
            break;
        }

        size_t imp_i, imp_j;
        double delta;
        bool is_penalty_entering;

        if (!findImprovingCell(problem, u, v, solution,
                              imp_i, imp_j, delta, is_penalty_entering, verbose)) {
            log("Улучшающих клеток не найдено -> решение оптимально", &solution.log, verbose);
            solution.is_optimal = true;
            break;
        }

        std::vector<std::pair<size_t, size_t>> cycle;
        if (!findCycle(solution, imp_i, imp_j, cycle, is_penalty_entering)) {
            log("Не удалось найти цикл", &solution.log, verbose);
            break;
        }

        if (verbose) {
            std::cout << "  Цикл: ";
            for (const auto& [i, j] : cycle) {
                std::cout << "(" << i+1 << "," << j+1 << ") ";
            }
            std::cout << "(длина=" << cycle.size() << ")\n";
        }

        std::pair<size_t, size_t> exiting_cell;
        double theta = redistributeAlongCycle(solution, cycle, exiting_cell, verbose);

        // Обновляем базис:
        // Все клетки цикла становятся базисными
        for (const auto& [i, j] : cycle) {
            solution.is_basic[i][j] = true;
        }
        // Выходящая клетка исключается из базиса
        if (exiting_cell.first < m && exiting_cell.second < n) {
            solution.is_basic[exiting_cell.first][exiting_cell.second] = false;
        }

        // Коррекция количества базисных переменных (должно быть m+n-1)
        size_t current_basis = solution.countBasicVars();
        if (current_basis < m + n - 1) {
            // Добавляем недостающие нулевые базисные (выбираем первые попавшиеся)
            for (size_t i = 0; i < m && solution.countBasicVars() < m + n - 1; ++i) {
                for (size_t j = 0; j < n && solution.countBasicVars() < m + n - 1; ++j) {
                    if (!solution.is_basic[i][j]) {
                        solution.is_basic[i][j] = true;
                        log(std::format("  [Вырожденность] Добавлена нулевая базисная x[{}][{}] = 0",
                                       i+1, j+1), &solution.log, verbose);
                        break;
                    }
                }
            }
        } else if (current_basis > m + n - 1) {
            // Такое не должно происходить, но на всякий случай удаляем лишние
            log("  [WARN] Лишние базисные клетки!", &solution.log, verbose);
        }

        // Пересчёт стоимости
        solution.transportation_cost = 0.0;
        for (size_t i = 0; i < m; ++i) {
            for (size_t j = 0; j < n; ++j) {
                solution.transportation_cost += problem.costs()[i][j] * solution.shipments[i][j];
            }
        }
        solution.total_cost = solution.transportation_cost;

        log(std::format("  Новая стоимость: {:.2f}", solution.total_cost),
            &solution.log, verbose);

        if (prev_cost - solution.total_cost < 1e-6) {
            if (++no_improvement_count > 5) {
                log("Нет улучшения стоимости 5 итераций подряд, завершаем", &solution.log, verbose);
                solution.is_optimal = true;
                break;
            }
        } else {
            no_improvement_count = 0;
        }
        prev_cost = solution.total_cost;

        if (theta < 1e-9) {
            if (++degenerate_count > m * n) {
                log("Слишком много вырожденных итераций, завершаем", &solution.log, verbose);
                solution.is_optimal = true;
                break;
            }
        } else {
            degenerate_count = 0;
        }
    }

    if (solution.iterations >= MAX_TOTAL_ITER) {
        log("Превышено максимальное число итераций", &solution.log, verbose);
    }

    log(std::format("Метод потенциалов завершён: итераций = {}, оптимально = {}",
                   solution.iterations, solution.is_optimal), &solution.log, verbose);

    return solution;
}

TransportSolution TransportSolver::solve(const TransportProblem& problem, bool verbose) {
    if (verbose) {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "  РЕШЕНИЕ ТРАНСПОРТНОЙ ЗАДАЧИ\n";
        std::cout << std::string(60, '=') << "\n";
    }

    TransportProblem expanded = createExpandedProblem(problem);

    if (verbose && problem.hasPenalties()) {
        std::cout << "[INFO] Создана расширенная задача с фиктивным поставщиком для штрафов\n";
        std::cout << "       Поставщиков: " << expanded.numSuppliers()
                  << " (оригинальных: " << problem.numSuppliers() << ")\n";
        std::cout << "       Потребителей: " << expanded.numConsumers()
                  << " (оригинальных: " << problem.numConsumers() << ")\n";
    }

    TransportSolution initial = solveNorthwestCorner(expanded, verbose);
    TransportSolution expanded_solution = solvePotentials(expanded, initial, verbose);

    TransportSolution original_solution;
    original_solution.shipments = extractOriginalPlan(
        expanded_solution.shipments,
        problem.numSuppliers(),
        problem.numConsumers()
    );

    original_solution.is_basic.assign(
        problem.numSuppliers(),
        std::vector<bool>(problem.numConsumers(), false)
    );
    for (size_t i = 0; i < problem.numSuppliers(); ++i) {
        for (size_t j = 0; j < problem.numConsumers(); ++j) {
            original_solution.is_basic[i][j] = (original_solution.shipments[i][j] > 1e-9);
        }
    }

    const size_t expected_basis = problem.numSuppliers() + problem.numConsumers() - 1;
    while (original_solution.countBasicVars() < expected_basis) {
        for (size_t i = 0; i < problem.numSuppliers() &&
             original_solution.countBasicVars() < expected_basis; ++i) {
            for (size_t j = 0; j < problem.numConsumers() &&
                 original_solution.countBasicVars() < expected_basis; ++j) {
                if (!original_solution.is_basic[i][j]) {
                    original_solution.is_basic[i][j] = true;
                    break;
                }
            }
        }
    }

    original_solution.transportation_cost = 0.0;
    for (size_t i = 0; i < problem.numSuppliers(); ++i) {
        for (size_t j = 0; j < problem.numConsumers(); ++j) {
            original_solution.transportation_cost +=
                problem.costs()[i][j] * original_solution.shipments[i][j];
        }
    }

    original_solution.penalty_cost = 0.0;
    if (problem.hasPenalties()) {
        const size_t dummy_idx = expanded.numSuppliers() - 1;
        for (size_t j = 0; j < problem.numConsumers(); ++j) {
            original_solution.penalty_cost +=
                expanded_solution.shipments[dummy_idx][j] * problem.penaltyRates()->at(j);
        }
    }

    original_solution.total_cost = original_solution.transportation_cost + original_solution.penalty_cost;
    original_solution.is_optimal = expanded_solution.is_optimal;
    original_solution.iterations = expanded_solution.iterations;
    original_solution.log = expanded_solution.log;

    if (verbose) {
        original_solution.print("ФИНАЛЬНОЕ РЕШЕНИЕ", &problem);
    }

    return original_solution;
}