#include "transport/TransportProblem.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>

constexpr static double EPS = 1e-9;

TransportProblem TransportProblem::fromFile(const std::string& filename) {
    TransportProblem problem;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    // Чтение m и n
    file >> problem.m >> problem.n;

    if (problem.m == 0 || problem.n == 0) {
        throw std::runtime_error("Invalid dimensions: m and n must be > 0");
    }

    // Чтение запасов
    problem.supply.resize(problem.m);
    for (size_t i = 0; i < problem.m; ++i) {
        file >> problem.supply[i];
        if (problem.supply[i] < 0) {
            throw std::runtime_error("Supply must be non-negative");
        }
    }

    // Чтение потребностей
    problem.demand.resize(problem.n);
    for (size_t j = 0; j < problem.n; ++j) {
        file >> problem.demand[j];
        if (problem.demand[j] < 0) {
            throw std::runtime_error("Demand must be non-negative");
        }
    }

    // Чтение матрицы стоимостей
    problem.cost.resize(problem.m, std::vector<double>(problem.n));
    for (size_t i = 0; i < problem.m; ++i) {
        for (size_t j = 0; j < problem.n; ++j) {
            file >> problem.cost[i][j];
        }
    }

    // Чтение штрафов за недопоставку (опционально)
    // Формат: после матрицы может идти одна строка с n штрафами
    problem.demandPenalty.resize(problem.n, 0.0);
    double firstPenalty;
    if (file >> firstPenalty) {
        problem.demandPenalty[0] = firstPenalty;
        for (size_t j = 1; j < problem.n; ++j) {
            file >> problem.demandPenalty[j];
        }
        
        // Чтение порогов недопоставки (опционально)
        problem.demandThreshold.resize(problem.n, 0.0);
        double firstThreshold;
        if (file >> firstThreshold) {
            problem.demandThreshold[0] = firstThreshold;
            for (size_t j = 1; j < problem.n; ++j) {
                file >> problem.demandThreshold[j];
            }
        }
    }

    return problem;
}

TransportProblem TransportProblem::fromConsole() {
    TransportProblem problem;

    std::cout << "Enter number of suppliers (m) and consumers (n): ";
    std::cin >> problem.m >> problem.n;

    if (problem.m == 0 || problem.n == 0) {
        throw std::runtime_error("Invalid dimensions: m and n must be > 0");
    }

    std::cout << "Enter " << problem.m << " supply values: ";
    problem.supply.resize(problem.m);
    for (size_t i = 0; i < problem.m; ++i) {
        std::cin >> problem.supply[i];
        if (problem.supply[i] < 0) {
            throw std::runtime_error("Supply must be non-negative");
        }
    }

    std::cout << "Enter " << problem.n << " demand values: ";
    problem.demand.resize(problem.n);
    for (size_t j = 0; j < problem.n; ++j) {
        std::cin >> problem.demand[j];
        if (problem.demand[j] < 0) {
            throw std::runtime_error("Demand must be non-negative");
        }
    }

    std::cout << "Enter cost matrix (" << problem.m << "x" << problem.n << "):" << std::endl;
    problem.cost.resize(problem.m, std::vector<double>(problem.n));
    for (size_t i = 0; i < problem.m; ++i) {
        for (size_t j = 0; j < problem.n; ++j) {
            std::cin >> problem.cost[i][j];
        }
    }

    std::cout << "Enter demand penalties (one per consumer, or press Enter for 0): " << std::endl;
    problem.demandPenalty.resize(problem.n, 0.0);
    std::string line;
    std::getline(std::cin, line);
    std::getline(std::cin, line);
    if (!line.empty()) {
        std::istringstream iss(line);
        for (size_t j = 0; j < problem.n; ++j) {
            if (!(iss >> problem.demandPenalty[j])) {
                problem.demandPenalty[j] = 0.0;
            }
        }
    }

    return problem;
}

bool TransportProblem::validate() const {
    if (m == 0 || n == 0) {
        std::cerr << "Error: Invalid dimensions" << std::endl;
        return false;
    }

    if (supply.size() != m || demand.size() != n) {
        std::cerr << "Error: Supply/demand vector size mismatch" << std::endl;
        return false;
    }

    if (cost.size() != m) {
        std::cerr << "Error: Cost matrix rows mismatch" << std::endl;
        return false;
    }

    for (size_t i = 0; i < m; ++i) {
        if (cost[i].size() != n) {
            std::cerr << "Error: Cost matrix columns mismatch" << std::endl;
            return false;
        }
    }

    if (!demandPenalty.empty() && demandPenalty.size() != n) {
        std::cerr << "Error: Demand penalty vector size mismatch" << std::endl;
        return false;
    }
    // Also check that penalty doesn't have negative values
    for (size_t j = 0; j < demandPenalty.size(); ++j) {
        if (demandPenalty[j] < 0) {
            std::cerr << "Error: Penalty must be non-negative" << std::endl;
            return false;
        }
    }

    double totalSupply = std::accumulate(supply.begin(), supply.end(), 0.0);
    double totalDemand = std::accumulate(demand.begin(), demand.end(), 0.0);

    if (totalSupply < EPS || totalDemand < EPS) {
        std::cerr << "Error: Total supply and demand must be positive" << std::endl;
        return false;
    }

    return true;
}

bool TransportProblem::isBalanced(double eps) const {
    double totalSupply = std::accumulate(supply.begin(), supply.end(), 0.0);
    double totalDemand = std::accumulate(demand.begin(), demand.end(), 0.0);
    return std::abs(totalSupply - totalDemand) < eps;
}

TransportProblem TransportProblem::balance() const {
    if (isBalanced()) {
        return *this;
    }

    TransportProblem balanced = *this;
    
    double totalSupply = std::accumulate(supply.begin(), supply.end(), 0.0);
    double totalDemand = std::accumulate(demand.begin(), demand.end(), 0.0);
    
    // Calculate total threshold
    double totalThreshold = 0.0;
    if (!demandThreshold.empty()) {
        totalThreshold = std::accumulate(demandThreshold.begin(), demandThreshold.end(), 0.0);
    }

    // Handle thresholds: add dummy supplier for free underdelivery (cost = 0)
    if (totalThreshold > 0) {
        balanced.supply.emplace_back(totalThreshold);
        std::vector<double> thresholdCost(balanced.n, 0.0);  // Zero cost for threshold
        balanced.cost.emplace_back(thresholdCost);
        balanced.m++;
    }
    
    // Now balance supply and demand (considering threshold dummy was added)
    totalSupply = std::accumulate(balanced.supply.begin(), balanced.supply.end(), 0.0);
    
    if (totalSupply > totalDemand) {
        // Add dummy consumer for excess supply
        balanced.demand.emplace_back(totalSupply - totalDemand);
        for (size_t i = 0; i < balanced.m; ++i) {
            balanced.cost[i].emplace_back(0.0);
        }
        balanced.n++;
    } else if (totalSupply < totalDemand) {
        // Need dummy supplier for remaining shortage (penalty applies)
        balanced.supply.emplace_back(totalDemand - totalSupply);
        std::vector<double> penaltyCost;
        if (demandPenalty.empty()) {
            penaltyCost = std::vector<double>(balanced.n, 0.0);
        } else {
            penaltyCost = demandPenalty;
        }
        balanced.cost.emplace_back(penaltyCost);
        balanced.m++;
    }
    // If totalSupply == totalDemand, no penalty dummy needed

    return balanced;
}

LinearProgram TransportProblem::toLinearProgram() const {
    if (!validate()) {
        throw std::runtime_error("Invalid transport problem");
    }

    // Проверяем баланс
    if (!isBalanced()) {
        throw std::runtime_error("Transport problem must be balanced. Call balance() first.");
    }

    size_t numVars = m * n;
    size_t numConstraints = m + n - 1; // Одно ограничение удаляем (линейная зависимость)

    // 1. Целевая функция: flatten cost matrix
    std::vector<double> objective(numVars);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            objective[i * n + j] = cost[i][j];
        }
    }

    // 2. Матрица ограничений A и вектор b
    std::vector<std::vector<double>> constraints(numConstraints, std::vector<double>(numVars, 0.0));
    std::vector<double> rhs(numConstraints);
    std::vector<std::string> relations(numConstraints, "="); // Все ограничения - равенства

    // Ограничения по поставщикам (m строк): sum_j x[i][j] = supply[i]
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            constraints[i][i * n + j] = 1.0;
        }
        rhs[i] = supply[i];
    }

    // Ограничения по потребителям (n-1 строк): sum_i x[i][j] = demand[j]
    // Последнее ограничение не добавляем (линейная зависимость)
    for (size_t j = 0; j < n - 1; ++j) {
        size_t row = m + j;
        for (size_t i = 0; i < m; ++i) {
            constraints[row][i * n + j] = 1.0;
        }
        rhs[row] = demand[j];
    }

    // 3. Ограничения на переменные: все x[i][j] >= 0
    std::vector<std::string> var_constraints(numVars, ">=0");

    // 4. Создаём задачу: минимизация
    return {
        true, // minimize = true
        objective, // c
        constraints, // A
        relations, // все "="
        rhs, // b
        var_constraints // все ">=0"
    };
}

void TransportProblem::print(const std::string& title) const {
    std::cout << "\n=== " << title << " ===" << std::endl;
    std::cout << "Suppliers (m): " << m << std::endl;
    std::cout << "Consumers (n): " << n << std::endl;

    std::cout << "\nSupply: ";
    for (size_t i = 0; i < m; ++i) {
        std::cout << "a" << (i + 1) << "=" << supply[i] << " ";
    }
    std::cout << "(total: " << std::accumulate(supply.begin(), supply.end(), 0.0) << ")" << std::endl;

    std::cout << "Demand: ";
    for (size_t j = 0; j < n; ++j) {
        std::cout << "b" << (j + 1) << "=" << demand[j] << " ";
    }
    std::cout << "(total: " << std::accumulate(demand.begin(), demand.end(), 0.0) << ")" << std::endl;

    std::cout << "\nCost matrix:" << std::endl;
    std::cout << "       ";
    for (size_t j = 0; j < n; ++j) {
        std::cout << std::setw(8) << ("B" + std::to_string(j + 1));
    }
    std::cout << std::endl;

    for (size_t i = 0; i < m; ++i) {
        std::cout << "A" << (i + 1) << "  ";
        for (size_t j = 0; j < n; ++j) {
            std::cout << std::setw(8) << cost[i][j];
        }
        std::cout << " | " << supply[i] << std::endl;
    }

    std::cout << "       ";
    for (size_t j = 0; j < n; ++j) {
        std::cout << std::setw(8) << demand[j];
    }
    std::cout << std::endl;
    std::cout << "========================\n" << std::endl;
}

void TransportProblem::printPlan(const std::vector<std::vector<double>>& plan) {
    size_t m = plan.size();
    if (m == 0) {
        std::cout << "Empty plan" << std::endl;
        return;
    }
    size_t n = plan[0].size();

    std::cout << "\nTransportation plan:" << std::endl;
    std::cout << "       ";
    for (size_t j = 0; j < n; ++j) {
        std::cout << std::setw(10) << ("B" + std::to_string(j + 1));
    }
    std::cout << std::endl;

    for (size_t i = 0; i < m; ++i) {
        std::cout << "A" << (i + 1) << "  ";
        for (size_t j = 0; j < n; ++j) {
            if (plan[i][j] > EPS) {
                std::cout << std::setw(10) << std::fixed << std::setprecision(2) << plan[i][j];
            } else {
                std::cout << std::setw(10) << "-";
            }
        }
        std::cout << std::endl;
    }
}

double TransportProblem::calculateCost(const std::vector<std::vector<double>>& plan) const {
    double total = 0.0;
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            total += cost[i][j] * plan[i][j];
        }
    }
    return total;
}

std::vector<std::vector<double>> TransportProblem::restorePlanFromVector(const std::vector<double>& x_vector, size_t m,
                                                                         size_t n) {
    if (x_vector.size() != m * n) {
        throw std::runtime_error("Vector size mismatch: expected " + std::to_string(m * n) + ", got " +
                                 std::to_string(x_vector.size()));
    }

    std::vector<std::vector<double>> plan(m, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            plan[i][j] = x_vector[i * n + j];
        }
    }
    return plan;
}
