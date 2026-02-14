#include "LinearProgram.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

LinearProgram::LinearProgram() : minimize_(true) {}

LinearProgram::LinearProgram(bool minimize, std::vector<double> objective, std::vector<std::vector<double>> constraints,
                             std::vector<std::string> relations, std::vector<double> rhs,
                             std::vector<std::string> var_constraints) :
    minimize_(minimize), objective_(std::move(objective)), constraints_(std::move(constraints)),
    relations_(std::move(relations)), rhs_(std::move(rhs)), var_constraints_(std::move(var_constraints)) {
    prettierCoeffs();
}

std::string format_coeff(double coeff, const std::string& var, bool first) {
    std::ostringstream oss;
    if (std::abs(coeff) < 1e-9)
        return "";

    if (!first && coeff > 0)
        oss << " + ";
    else if (!first && coeff < 0)
        oss << " - ";
    else if (first && coeff < 0)
        oss << "-";

    double abs_coeff = std::abs(coeff);
    if (std::abs(abs_coeff - 1.0) < 1e-9)
        oss << var;
    else
        oss << std::fixed << std::setprecision(2) << abs_coeff << "*" << var;

    return oss.str();
}

void print_objective(const LinearProgram& lp, const std::string& prefix) {
    std::cout << prefix;
    std::cout << (lp.is_minimization() ? "minimize: " : "maximize: ");

    bool first = true;
    size_t n = lp.num_variables();
    for (int i = 0; i < n; ++i) {
        std::string term = format_coeff(lp.objective()[i], "x" + std::to_string(i + 1), first);
        if (!term.empty()) {
            std::cout << term;
            first = false;
        }
    }
    if (first)
        std::cout << "0";
    std::cout << std::endl;
}

void print_constraints(const LinearProgram& lp, const std::string& prefix) {
    size_t m = lp.num_constraints();
    size_t n = lp.num_variables();

    for (int i = 0; i < m; ++i) {
        std::cout << prefix;
        bool first = true;
        for (int j = 0; j < n; ++j) {
            std::string term = format_coeff(lp.constraints()[i][j], "x" + std::to_string(j + 1), first);
            if (!term.empty()) {
                std::cout << term;
                first = false;
            }
        }
        if (first)
            std::cout << "0"; // All coefficients zero
        std::cout << " " << lp.relations()[i] << " " << std::fixed << std::setprecision(2) << lp.rhs()[i] << std::endl;
    }
}

void print_variable_constraints(const LinearProgram& lp, const std::string& prefix) {
    size_t n = lp.num_variables();
    std::cout << prefix << "subject to:" << std::endl;
    for (size_t i = 0; i < n; ++i) {
        std::cout << prefix << "  x" << (i + 1) << " " << lp.var_constraints()[i] << std::endl;
    }
}

void LinearProgram::print(const std::string& title) const {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << title << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    print_objective(*this, "  ");
    std::cout << "  subject to:" << std::endl;
    print_constraints(*this, "    ");
    print_variable_constraints(*this, "    ");
    std::cout << "  form: " << FormToString(this->getForm()) << std::endl;
}

// Read from file
LinearProgram LinearProgram::read_from_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: cannot open file '" << filename << "'" << std::endl;
        std::exit(1);
    }

    std::string line;

    // Read problem type
    if (!std::getline(file, line) || (line != "min" && line != "max")) {
        std::cerr << "Error: first line must be 'min' or 'max'" << std::endl;
        std::exit(1);
    }
    bool minimize = (line == "min");

    if (!std::getline(file, line)) {
        std::cerr << "Error: first line must be 'min' or 'max'" << std::endl;
        std::exit(1);
    }
    size_t n = std::stoi(line);

    // Read objective coefficients
    if (!std::getline(file, line)) {
        std::cerr << "Error: missing objective function coefficients" << std::endl;
        std::exit(1);
    }
    std::istringstream obj_stream(line);
    std::vector<double> objective(n);
    for (int i = 0; i < n; ++i) {
        if (!(obj_stream >> objective[i])) {
            std::cerr << "Error: invalid objective function coefficients" << std::endl;
            std::exit(1);
        }
    }

    // Read number of constraints
    int m;
    if (!(file >> m) || m < 1) {
        std::cerr << "Error: invalid number of constraints (must be positive)" << std::endl;
        std::exit(1);
    }
    file.ignore(); // Skip newline

    std::vector<std::vector<double>> constraints(m, std::vector<double>(n));
    std::vector<std::string> relations(m);
    std::vector<double> rhs(m);

    // Read constraints
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!(file >> constraints[i][j])) {
                std::cerr << "Error: missing coefficients in constraint " << (i + 1) << std::endl;
                std::exit(1);
            }
        }

        if (!(file >> relations[i]) || !is_valid_constraint_type(relations[i])) {
            std::cerr << "Error: invalid constraint type '" << relations[i] << "' in constraint " << (i + 1)
                      << std::endl;
            std::exit(1);
        }

        if (!(file >> rhs[i])) {
            std::cerr << "Error: missing RHS value in constraint " << (i + 1) << std::endl;
            std::exit(1);
        }
    }

    // Read variable constraints
    std::vector<std::string> var_constraints(n);
    for (int i = 0; i < n; ++i) {
        if (!(file >> var_constraints[i]) || !is_valid_variable_constraint(var_constraints[i])) {
            std::cerr << "Error: invalid constraint '" << var_constraints[i] << "' for variable x" << (i + 1)
                      << std::endl;
            std::exit(1);
        }
    }

    return {minimize, objective, constraints, relations, rhs, var_constraints};
}

// Read from console
LinearProgram LinearProgram::read_from_console() {
    std::string obj_type;

    std::cout << "Enter problem type (min/max): ";
    std::cin >> obj_type;
    if (obj_type != "min" && obj_type != "max") {
        std::cerr << "Error: problem type must be 'min' or 'max'" << std::endl;
        std::exit(1);
    }
    bool minimize = (obj_type == "min");

    std::cout << "Enter num of variables: ";
    int n = 0;
    std::cin >> n;
    std::cout << "Enter " << n << " objective function coefficients (space-separated):" << std::endl;
    std::vector<double> objective(n);
    for (int i = 0; i < n; ++i) {
        if (!(std::cin >> objective[i])) {
            std::cerr << "Error: invalid objective coefficients" << std::endl;
            std::exit(1);
        }
    }

    std::cout << "Enter number of constraints: ";
    int m;
    std::cin >> m;
    if (m < 1) {
        std::cerr << "Error: number of constraints must be positive" << std::endl;
        std::exit(1);
    }

    std::vector<std::vector<double>> constraints(m, std::vector<double>(n));
    std::vector<std::string> relations(m);
    std::vector<double> rhs(m);

    std::cout << "Enter constraints (each: " << n << " coefficients, type <=/>=/=, RHS value):" << std::endl;
    for (int i = 0; i < m; ++i) {
        std::cout << "Constraint " << (i + 1) << ": ";
        for (int j = 0; j < n; ++j) {
            if (!(std::cin >> constraints[i][j])) {
                std::cerr << "Error: invalid coefficients in constraint " << (i + 1) << std::endl;
                std::exit(1);
            }
        }

        std::cin >> relations[i];
        if (!is_valid_constraint_type(relations[i])) {
            std::cerr << "Error: invalid constraint type '" << relations[i] << "' in constraint " << (i + 1)
                      << std::endl;
            std::exit(1);
        }

        std::cin >> rhs[i];
    }

    std::cout << "Enter variable constraints (x1..x5):" << std::endl;
    std::cout << "Available values: >=0, <=0, free" << std::endl;
    std::vector<std::string> var_constraints(n);
    for (int i = 0; i < n; ++i) {
        std::cout << "x" << (i + 1) << " = ";
        std::cin >> var_constraints[i];
        if (!is_valid_variable_constraint(var_constraints[i])) {
            std::cerr << "Error: invalid constraint '" << var_constraints[i] << "' for variable x" << (i + 1)
                      << std::endl;
            std::exit(1);
        }
    }

    return {minimize, objective, constraints, relations, rhs, var_constraints};
}

// Validation helpers
bool LinearProgram::is_valid_constraint_type(const std::string& rel) {
    return (rel == "<=" || rel == ">=" || rel == "=");
}

bool LinearProgram::is_valid_variable_constraint(const std::string& constraint) {
    return (constraint == ">=0" || constraint == "<=0" || constraint == "free");
}
