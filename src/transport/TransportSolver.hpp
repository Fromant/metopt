#pragma once

#include "TransportProblem.hpp"
#include <vector>
#include <string>

struct TransportSolution {
    std::vector<std::vector<double>> shipments;
    std::vector<std::vector<bool>> is_basic;
    double total_cost{0.0};
    double transportation_cost{0.0};
    double penalty_cost{0.0};
    bool is_optimal{false};
    size_t iterations{0};
    std::vector<std::string> log;

    void print(const std::string& title = "Solution",
               const TransportProblem* problem = nullptr) const;
    double getActualDelivery(size_t consumer_idx, const TransportProblem& problem) const;

    [[nodiscard]] size_t countBasicVars() const {
        size_t count = 0;
        for (const auto & i : is_basic)
            for (bool j : i)
                if (j) ++count;
        return count;
    }

    [[nodiscard]] double getActualDelivery(size_t consumer_idx) const {
        double sum = 0.0;
        for (const auto & shipment : shipments) {
            if (consumer_idx < shipment.size()) {
                sum += shipment[consumer_idx];
            }
        }
        return sum;
    }
};

class TransportSolver {
public:
    static TransportSolution solveNorthwestCorner(const TransportProblem& problem,
                                                  bool verbose = true);

    static TransportSolution solvePotentials(const TransportProblem& problem,
                                            const TransportSolution& initial_solution,
                                            bool verbose = true);

    static TransportSolution solve(const TransportProblem& problem,
                                  bool verbose = true);

private:
    static bool calculatePotentials(const TransportProblem& problem,
                                   const TransportSolution& solution,
                                   std::vector<double>& u,
                                   std::vector<double>& v,
                                   bool verbose);

    static bool findImprovingCell(const TransportProblem& problem,
                                 const std::vector<double>& u,
                                 const std::vector<double>& v,
                                 const TransportSolution& solution,
                                 size_t& out_i, size_t& out_j,
                                 double& out_delta,
                                 bool verbose);
    static bool findCycleDFS(size_t i, size_t j, bool horizontal, size_t start_i, size_t start_j,
                      const std::vector<std::vector<bool>>& is_basic, std::vector<std::pair<size_t, size_t>>& path,
                      std::vector<std::vector<bool>>& visited, std::vector<std::pair<size_t, size_t>>& cycle);

    static bool findCycle(const TransportSolution& solution,
                         size_t start_i, size_t start_j,
                         std::vector<std::pair<size_t, size_t>>& cycle);

    static void redistributeAlongCycle(TransportSolution& solution,
                                      const std::vector<std::pair<size_t, size_t>>& cycle,
                                      double theta);

    static void log(const std::string& message,
                   std::vector<std::string>* log_ptr,
                   bool verbose);
};