#include "TransportProblem.hpp"
#include <cmath>
#include <format>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>

// ==================== Конструктор и валидация ====================

TransportProblem::TransportProblem(std::vector<double> supplies, std::vector<double> demands,
                                   std::vector<std::vector<double>> costs) :
    supplies_(std::move(supplies)), demands_(std::move(demands)), costs_(std::move(costs)) {
    validate();
}

void TransportProblem::validate() const {
    if (supplies_.empty() || demands_.empty()) {
        throw std::invalid_argument("Supplies and demands cannot be empty");
    }
    if (costs_.size() != supplies_.size()) {
        throw std::invalid_argument("Cost matrix rows must match number of suppliers");
    }
    for (const auto& cost : costs_) {
        if (cost.size() != demands_.size()) {
            throw std::invalid_argument("Cost matrix columns must match number of consumers");
        }
    }

    if (penalty_thresholds_.has_value() != penalty_rates_.has_value()) {
        throw std::invalid_argument("Penalty thresholds and rates must be both set or both unset");
    }
    if (penalty_thresholds_.has_value()) {
        if (penalty_thresholds_->size() != demands_.size() || penalty_rates_->size() != demands_.size()) {
            throw std::invalid_argument("Penalty vectors must match number of consumers");
        }
    }
}

void TransportProblem::setPenalties(std::vector<double> thresholds, std::vector<double> rates) {
    if (thresholds.size() != demands_.size() || rates.size() != demands_.size()) {
        throw std::invalid_argument("Penalty vectors must match number of consumers");
    }
    penalty_thresholds_ = std::move(thresholds);
    penalty_rates_ = std::move(rates);
    validate();
}

bool TransportProblem::isBalanced(double epsilon) const { return std::abs(totalSupply() - totalDemand()) < epsilon; }

double TransportProblem::totalSupply() const { return std::accumulate(supplies_.begin(), supplies_.end(), 0.0); }

double TransportProblem::totalDemand() const { return std::accumulate(demands_.begin(), demands_.end(), 0.0); }

TransportProblem TransportProblem::balance() const {
    if (isBalanced()) {
        return *this;
    }

    TransportProblem balanced = *this;
    const double supply = totalSupply();
    const double demand = totalDemand();

    if (supply > demand) {
        balanced.demands_.emplace_back(supply - demand);
        for (auto& row : balanced.costs_) {
            row.emplace_back(0.0);
        }
        if (balanced.penalty_thresholds_) {
            balanced.penalty_thresholds_->emplace_back(0.0);
            balanced.penalty_rates_->emplace_back(0.0);
        }
    } else {
        balanced.supplies_.emplace_back(demand - supply);
        balanced.costs_.emplace_back(balanced.demands_.size(), 0.0);
    }

    balanced.validate();
    return balanced;
}

double TransportProblem::calculatePenaltyCost(const std::vector<std::vector<double>>& shipments) const {
    if (!hasPenalties())
        return 0.0;

    const size_t n = demands_.size();
    const size_t m = shipments.size();
    double total_penalty = 0.0;

    for (size_t j = 0; j < n; ++j) {
        double delivered = 0.0;
        for (size_t i = 0; i < m; ++i) {
            if (j < shipments[i].size()) {
                delivered += shipments[i][j];
            }
        }

        const double threshold = penalty_thresholds_->at(j);
        const double shortage = std::max(0.0, demands_[j] - threshold - delivered);
        total_penalty += shortage * penalty_rates_->at(j);
    }

    return total_penalty;
}

TransportProblem TransportProblem::createExpandedProblem(const TransportProblem& original) {
    if (!original.hasPenalties()) {
        return original.balance(); // если штрафов нет, просто балансируем
    }

    const size_t m = original.numSuppliers();
    const size_t n = original.numConsumers();

    const size_t m_expanded = m + 1;
    const size_t n_expanded = n + 1;

    std::vector<double> supplies(m_expanded);
    std::vector<double> demands(n_expanded);
    std::vector<std::vector<double>> costs(m_expanded, std::vector<double>(n_expanded, 0.0));

    for (size_t i = 0; i < m; ++i) {
        supplies[i] = original.supplies_[i];
    }

    double dummy_supply = 0.0;
    for (size_t j = 0; j < n; ++j) {
        double max_shortage = original.demands_[j] - original.penalty_thresholds_->at(j);
        dummy_supply += std::max(0.0, max_shortage);
    }
    supplies[m] = dummy_supply;

    for (size_t j = 0; j < n; ++j) {
        demands[j] = original.demands_[j];
    }

    demands[n] = original.totalSupply() + dummy_supply - original.totalDemand();

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            costs[i][j] = original.costs_[i][j];
        }
        costs[i][n] = 0.0;
    }

    for (size_t j = 0; j < n; ++j) {
        costs[m][j] = original.penalty_rates_->at(j);
    }
    costs[m][n] = 0.0;

    return {supplies, demands, costs};
}

TransportProblem TransportProblem::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    size_t m, n;
    file >> m >> n;

    std::vector<double> supplies(m);
    for (auto& s : supplies)
        file >> s;

    std::vector<double> demands(n);
    for (auto& d : demands)
        file >> d;

    std::vector<std::vector<double>> costs(m, std::vector<double>(n));
    for (auto& row : costs) {
        for (auto& c : row)
            file >> c;
    }

    TransportProblem problem(supplies, demands, costs);

    // Читаем пенальти если есть
    std::string marker;
    if (file >> marker && marker == "PENALTIES") {
        std::vector<double> thresholds(n), rates(n);
        for (auto& t : thresholds)
            file >> t;
        for (auto& r : rates)
            file >> r;
        problem.setPenalties(std::move(thresholds), std::move(rates));
    }

    return problem;
}

TransportProblem TransportProblem::readFromConsole() {
    std::cout << "=== Ввод транспортной задачи ===\n";

    size_t m, n;
    std::cout << "Число поставщиков (m): ";
    std::cin >> m;
    std::cout << "Число потребителей (n): ";
    std::cin >> n;

    std::vector<double> supplies(m);
    std::cout << "Запасы поставщиков:\n";
    for (size_t i = 0; i < m; ++i) {
        std::cout << "  A[" << i + 1 << "]: ";
        std::cin >> supplies[i];
    }

    std::vector<double> demands(n);
    std::cout << "Потребности потребителей:\n";
    for (size_t j = 0; j < n; ++j) {
        std::cout << "  B[" << j + 1 << "]: ";
        std::cin >> demands[j];
    }

    std::vector<std::vector<double>> costs(m, std::vector<double>(n));
    std::cout << "Матрица стоимостей:\n";
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            std::cout << "  c[" << i + 1 << "][" << j + 1 << "]: ";
            std::cin >> costs[i][j];
        }
    }

    TransportProblem problem(supplies, demands, costs);

    char add_penalties;
    std::cout << "Добавить штрафы за недопоставку? (y/n): ";
    std::cin >> add_penalties;
    if (add_penalties == 'y' || add_penalties == 'Y') {
        std::vector<double> thresholds(n), rates(n);
        std::cout << "Пороги недопоставки:\n";
        for (size_t j = 0; j < n; ++j) {
            std::cout << "  Threshold[" << j + 1 << "]: ";
            std::cin >> thresholds[j];
        }
        std::cout << "Ставки штрафов:\n";
        for (size_t j = 0; j < n; ++j) {
            std::cout << "  PenaltyRate[" << j + 1 << "]: ";
            std::cin >> rates[j];
        }
        problem.setPenalties(std::move(thresholds), std::move(rates));
    }

    return problem;
}

LinearProgram TransportProblem::toLinearProgram() const {
    const size_t m = numSuppliers();
    const size_t n = numConsumers();

    const size_t num_vars = m * n; // только переменные x_ij

    std::vector<double> objective(num_vars, 0.0);
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j < n; ++j)
            objective[i * n + j] = costs_[i][j];

    std::vector<std::vector<double>> constraints;
    std::vector<std::string> relations;
    std::vector<double> rhs;

    // Ограничения поставщиков (все m)
    for (size_t i = 0; i < m; ++i) {
        std::vector<double> row(num_vars, 0.0);
        for (size_t j = 0; j < n; ++j)
            row[i * n + j] = 1.0;
        constraints.push_back(std::move(row));
        relations.push_back("=");
        rhs.push_back(supplies_[i]);
    }

    // Ограничения потребителей (первые n-1, последнее исключаем)
    for (size_t j = 0; j < n - 1; ++j) {
        std::vector<double> row(num_vars, 0.0);
        for (size_t i = 0; i < m; ++i)
            row[i * n + j] = 1.0;
        constraints.push_back(std::move(row));
        relations.push_back("=");
        rhs.push_back(demands_[j]);
    }

    std::vector<std::string> var_constraints(num_vars, ">=0");

    auto lp = LinearProgram(true, std::move(objective), std::move(constraints), std::move(relations), std::move(rhs),
                            std::move(var_constraints));
    lp.prettierCoeffs();
    return lp;
}

std::vector<std::vector<double>> TransportProblem::restorePlanFromVector(const std::vector<double>& lp_solution,
                                                                         size_t original_m, size_t original_n,
                                                                         size_t expanded_m, size_t expanded_n) {
    std::vector<std::vector<double>> plan(original_m, std::vector<double>(original_n, 0.0));
    const size_t num_x_vars = expanded_m * expanded_n;

    if (lp_solution.size() < num_x_vars) {
        throw std::invalid_argument("LP solution vector too small");
    }

    for (size_t i = 0; i < original_m; ++i) {
        for (size_t j = 0; j < original_n; ++j) {
            const size_t idx = i * expanded_n + j; // позиция в расширенной матрице
            if (idx < lp_solution.size()) {
                plan[i][j] = std::max(0.0, lp_solution[idx]);
            }
        }
    }

    return plan;
}

bool TransportProblem::validatePlan(const std::vector<std::vector<double>>& plan, const std::vector<double>& supplies,
                                    const std::vector<double>& demands, double epsilon) {
    if (plan.size() != supplies.size())
        return false;
    if (plan.empty() || plan[0].size() != demands.size())
        return false;

    for (const auto& row : plan)
        for (double val : row)
            if (val < -epsilon)
                return false;

    for (size_t i = 0; i < supplies.size(); ++i) {
        double sum = 0;
        for (double val : plan[i])
            sum += val;
        if (std::abs(sum - supplies[i]) > epsilon)
            return false;
    }

    const size_t n = plan[0].size();
    for (size_t j = 0; j < n; ++j) {
        double sum = 0;
        for (const auto& i : plan)
            sum += i[j];
        if (std::abs(sum - demands[j]) > epsilon)
            return false;
    }

    return true;
}

void TransportProblem::printPlan(const std::vector<std::vector<double>>& plan, const std::vector<double>& supplies,
                                 const std::vector<double>& demands, const std::string& title) {
    std::cout << "\n" << std::string(60, '-') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << std::fixed << std::setprecision(2);

    if (plan.empty()) {
        std::cout << "  [Empty plan]\n";
        return;
    }

    const size_t m = plan.size();
    const size_t n = plan[0].size();

    std::cout << "             ";
    for (size_t j = 0; j < n; ++j)
        std::cout << std::format("B{:2d}     ", j + 1);
    std::cout << "Supply\n";

    for (size_t i = 0; i < m; ++i) {
        std::cout << std::format("Supplier {:2d}: ", i + 1);
        double row_sum = 0.0;

        for (size_t j = 0; j < n; ++j) {
            const double val = plan[i][j];
            row_sum += val;
            if (val > 1e-9) {
                std::cout << std::format("{:5.2f}* ", val);
            } else {
                std::cout << "  .    ";
            }
        }
        std::cout << std::format(" | {:6.2f}", row_sum);
        if (i < supplies.size()) {
            std::cout << std::format(" (avail: {:.2f})", supplies[i]);
        }
        std::cout << "\n";
    }

    std::cout << "Demand:      ";
    for (size_t j = 0; j < n; ++j) {
        double col_sum = 0.0;
        for (size_t i = 0; i < m; ++i)
            col_sum += plan[i][j];
        std::cout << std::format("{:5.2f}  ", col_sum);
    }
    std::cout << "\n";

    std::cout << std::string(60, '-') << "\n";
}

void TransportProblem::print(const std::string& title) const {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '=') << "\n";

    std::cout << "Поставщики (" << numSuppliers() << "): ";
    for (double s : supplies_)
        std::cout << std::format("{:.2f} ", s);

    std::cout << "\nПотребители (" << numConsumers() << "): ";
    for (double d : demands_)
        std::cout << std::format("{:.2f} ", d);

    std::cout << "\nСуммарный запас: " << totalSupply() << ", спрос: " << totalDemand() << " ["
              << (isBalanced() ? "сбалансирована" : "НЕ сбалансирована") << "]\n";

    if (hasPenalties()) {
        std::cout << "\nШТРАФЫ ЗА НЕДОПОСТАВКУ:\n";
        std::cout << "  Потребитель | Порог | Ставка | Макс.штраф\n";
        std::cout << "  ------------|-------|--------|-----------\n";
        for (size_t j = 0; j < demands_.size(); ++j) {
            double max_penalty = demands_[j] * penalty_rates_->at(j);
            std::cout << std::format("  B{:>2d}        | {:>5.0f} | {:>6.0f} | {:>9.2f}\n", j + 1,
                                     penalty_thresholds_->at(j), penalty_rates_->at(j), max_penalty);
        }
    }

    std::cout << "\nМатрица стоимостей c[i][j]:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "        ";
    for (size_t j = 0; j < demands_.size(); ++j)
        std::cout << "B" << std::setw(4) << j + 1 << " ";
    std::cout << "\n";

    for (size_t i = 0; i < costs_.size(); ++i) {
        std::cout << "A" << std::setw(2) << i + 1 << ":   ";
        for (double cost : costs_[i]) {
            std::cout << std::setw(7) << cost << " ";
        }
        std::cout << "\n";
    }
    std::cout << std::string(60, '=') << "\n";
}
