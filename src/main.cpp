#include "solver.hpp"
#include "logical_constraints.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <numeric>

using namespace bilp;

int main() {
    std::cout << "=== BILP Solver CLI ===\n\n";

    int n;
    std::cout << "Number of projects: ";
    if (!(std::cin >> n) || n <= 0) { std::cerr << "Invalid.\n"; return 1; }
    std::cin.ignore(32767, '\n');

    double budget;
    std::cout << "Budget: ";
    if (!(std::cin >> budget)) { std::cerr << "Invalid.\n"; return 1; }
    std::cin.ignore(32767, '\n');

    std::vector<double> costs(n), returns(n);
    std::cout << "Costs (space-separated): ";
    for (int i = 0; i < n; ++i) std::cin >> costs[i];
    std::cin.ignore(32767, '\n');

    std::cout << "Returns (space-separated): ";
    for (int i = 0; i < n; ++i) std::cin >> returns[i];
    std::cin.ignore(32767, '\n');

    int aux_count = 0;
    std::cout << "Max auxiliary variables: ";
    std::cin >> aux_count;
    std::cin.ignore(32767, '\n');
    int total_vars = n + aux_count;

    SolverConfig config;
    config.n = n; config.budget = budget;
    config.costs = costs; config.returns = returns;
    config.total_vars = total_vars;
    config.primary_vars.resize(n);
    std::iota(config.primary_vars.begin(), config.primary_vars.end(), 0);
    config.aux_vars.reserve(aux_count);

    LinearConstraints lc;
    int next_aux = n + 1;

    std::cout << "\nEnter constraints (type 'done' to finish):\n";
    std::cout << "  implies <ant> <c1> <c2> ...\n";
    std::cout << "  exactly <k1> <v1> <v2> ... -> <k2> <w1> <w2> ...\n";

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty() || line == "done") break;
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd == "implies") {
            int ant;
            std::vector<int> cons;
            if (!(iss >> ant)) { std::cout << "  Error: bad format\n"; continue; }
            int v; while (iss >> v) cons.push_back(v);
            auto rule = logic::rule_implies_or(ant, cons, total_vars);
            logic::add_constraints(lc, rule, total_vars);
        } else if (cmd == "exactly") {
            int k1, k2;
            std::vector<int> vars1, vars2;
            if (!(iss >> k1)) { std::cout << "  Error: missing k1\n"; continue; }
            std::string token;
            bool arrow = false;
            while (iss >> token && token != "->") {
                try { vars1.push_back(std::stoi(token)); } catch(...) {}
            }
            if (token != "->") { std::cout << "  Error: missing '->'\n"; continue; }
            if (!(iss >> k2)) { std::cout << "  Error: missing k2\n"; continue; }
            while (iss >> token) {
                try { vars2.push_back(std::stoi(token)); } catch(...) {}
            }
            if (vars1.empty() || vars2.empty()) { std::cout << "  Error: missing vars\n"; continue; }
            if (next_aux + 1 > total_vars) { std::cout << "  Error: increase aux count\n"; continue; }
            int aux1 = next_aux++, aux2 = next_aux++;
            config.aux_vars.push_back(aux1 - 1); config.aux_vars.push_back(aux2 - 1);
            auto rule = logic::rule_exactly_implies_exactly(vars1, k1, vars2, k2, aux1, aux2, total_vars);
            logic::add_constraints(lc, rule, total_vars);
        } else {
            std::cout << "  Unknown command\n";
        }
    }

    config.constraints = lc;

    std::cout << "\nSolving...\n";
    Solver solver(config);
    SolverResult result = solver.solve();

    std::cout << "\n=== Results ===\n";
    std::cout << "Feasible: " << (result.feasible ? "Yes" : "No") << "\n";
    std::cout << "Max Profit (NPV): " << std::fixed << std::setprecision(2) << result.optimal_npv << "\n";
    std::cout << "Selected projects (1-based): ";
    bool first = true;
    for (int i = 0; i < n; ++i) {
        if (result.solution[i] == 1) {
            if (!first) std::cout << ", ";
            std::cout << (i + 1); first = false;
        }
    }
    if (first) std::cout << "none";
    std::cout << "\nNodes explored: " << result.nodes_explored << "\n";
    double used = 0.0;
    for (int i = 0; i < n; ++i) if (result.solution[i]) used += costs[i];
    std::cout << "Budget used: " << std::fixed << std::setprecision(2) << used << " / " << budget << "\n";

    return 0;
}