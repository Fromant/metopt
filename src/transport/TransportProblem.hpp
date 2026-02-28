#pragma once

#include <iomanip>
#include <optional>
#include <string>
#include <vector>
#include "linear/LinearProgram.hpp"

class TransportProblem {
private:
    std::vector<double> supplies_;
    std::vector<double> demands_;
    std::vector<std::vector<double>> costs_;
    std::optional<std::vector<double>> penalty_thresholds_;
    std::optional<std::vector<double>> penalty_rates_;

    void validate() const;

public:
    TransportProblem() = default;

    TransportProblem(std::vector<double> supplies, std::vector<double> demands, std::vector<std::vector<double>> costs);

    void setPenalties(std::vector<double> thresholds, std::vector<double> rates);

    static TransportProblem readFromFile(const std::string& filename);
    static TransportProblem readFromConsole();

    [[nodiscard]] const auto& supplies() const { return supplies_; }
    [[nodiscard]] const auto& demands() const { return demands_; }
    [[nodiscard]] const auto& costs() const { return costs_; }
    [[nodiscard]] const auto& penaltyThresholds() const { return penalty_thresholds_; }
    [[nodiscard]] const auto& penaltyRates() const { return penalty_rates_; }

    [[nodiscard]] size_t numSuppliers() const { return supplies_.size(); }
    [[nodiscard]] size_t numConsumers() const { return demands_.size(); }
    [[nodiscard]] bool hasPenalties() const { return penalty_thresholds_.has_value() && penalty_rates_.has_value(); }

    [[nodiscard]] bool isBalanced(double epsilon = 1e-9) const;
    [[nodiscard]] double totalSupply() const;
    [[nodiscard]] double totalDemand() const;

    [[nodiscard]] TransportProblem balance() const;
    [[nodiscard]] LinearProgram toLinearProgram() const;
    [[nodiscard]] double calculatePenaltyCost(const std::vector<std::vector<double>>& shipments) const;

    void print(const std::string& title = "Transport Problem") const;

    // Static helpers
    [[nodiscard]] static std::vector<std::vector<double>> restorePlanFromVector(const std::vector<double>& lp_solution,
                                                                                size_t original_m, size_t original_n,
                                                                                size_t balanced_m, size_t balanced_n,
                                                                                bool has_penalties);

    [[nodiscard]] static bool validatePlan(const std::vector<std::vector<double>>& plan,
                                           const std::vector<double>& supplies, const std::vector<double>& demands,
                                           double epsilon = 1e-6);

    static void printPlan(const std::vector<std::vector<double>>& plan, const std::vector<double>& supplies,
                          const std::vector<double>& demands, const std::string& title = "Transportation Plan");
};
