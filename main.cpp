#include <iostream>
#include <string>

#include "lib.hpp"
#include "src/DualBuilder.hpp"
#include "src/FormConverter.hpp"
#include "src/LinearProgram.hpp"
#include "src/Solvers/EnumSolver.hpp"

#include "src/Solvers/SimplexSolver.hpp"

void print_simplex_solution(const SimplexSolver::Solution& res) {
    const auto print_status = [&res]() {
        if (!res.status_message.empty()) {
            std::cout << "Status: " << res.status_message;
        }
        std::cout << std::endl;
    };

    if (!res.is_feasible) {
        std::cout << "Simplex solver failed: task is infeasible" << std::endl;
        print_status();
        return;
    }

    if (res.is_unbounded) {
        std::cout << "Simplex solver failed: task is unbounded" << std::endl;
        print_status();
        return;
    }

    if (!res.is_optimal) {
        std::cout << "Simplex solver failed: result is not optimal" << std::endl;
        print_status();
    }
    std::cout << "Simplex solver solution:" << std::endl;
    std::cout << "\tObjective value: " << res.objective_value << std::endl;
    std::cout << "\tx: ";
    print_vector(res.x);
    std::cout << "\tbasis: ";
    print_vector(res.basis);
    print_status();
}

void print_enum_solution(const EnumSolver::Solution& res) {
    const auto print_status = [&res]() {
        if (!res.status_message.empty()) {
            std::cout << "Status: " << res.status_message;
        }
        std::cout << std::endl;
    };

    if (!res.is_feasible) {
        std::cout << "Enum solver failed: task is infeasible" << std::endl;
        print_status();
        return;
    }

    if (res.is_unbounded) {
        std::cout << "Enum solver failed: task is unbounded" << std::endl;
        print_status();
        return;
    }

    if (!res.is_optimal) {
        std::cout << "Enum solver failed: result is not optimal" << std::endl;
        print_status();
    }
    std::cout << "Enum solver solution:" << std::endl;
    std::cout << "\tObjective value: " << res.objective_value << std::endl;
    std::cout << "\tx: ";
    print_vector(res.x);
    std::cout << "\tbasis: ";
    print_vector(res.basis);
    print_status();
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(2);

    // Determine input source
    LinearProgram original;
    if (argc > 1) {
        // Read from file specified in command-line argument
        std::string filename = argv[1];
        std::cout << "\nReading problem from file: " << filename << std::endl;
        original = LinearProgram::read_from_file(filename);
    } else {
        // Read from console
        std::cout << "\nNo input file specified. Reading from console..." << std::endl;
        original = LinearProgram::read_from_console();
    }

    original.print("Input problem:");

    const auto print_forms = [&original]() {
        LinearProgram general = FormConverter::to_general_form(original);
        general.print("GENERAL FORM (formula 4.1)");
        LinearProgram dual_general = DualBuilder::build_dual(general);
        dual_general.print("DUAL PROBLEM FOR GENERAL FORM");

        LinearProgram symmetric = FormConverter::to_symmetric_form(original);
        symmetric.print("SYMMETRIC FORM (formula 4.2)");
        LinearProgram dual_symmetric = DualBuilder::build_dual(symmetric);
        dual_symmetric.print("DUAL PROBLEM FOR SYMMETRIC FORM");

        LinearProgram canonical = FormConverter::to_canonical_form(original);
        canonical.print("CANONICAL FORM (formula 4.3)");
        LinearProgram dual_canonical = DualBuilder::build_dual(canonical);
        dual_canonical.print("DUAL PROBLEM FOR CANONICAL FORM");
    };

    // print_forms();

    const auto print_obj_GOST = [](const double val1, const double val2) {
        double err = std::abs(val1 - val2) / 2;
        double avg = (val1 + val2) / 2;

        const double exponent = std::floor(std::log10(err));
        // Разряд последней значащей цифры погрешности: -exponent
        const auto precision = std::clamp(-exponent, 0.0, 10.0);
        const double factor = std::pow(10.0, -precision - 1); // Сохраняем 2 значащие цифры
        err = std::round(err / factor) * factor;
        avg = std::round(avg / factor) * factor;
        std::cout << std::fixed << std::setprecision(precision) << avg << " +- " << err << std::endl;
        const double rel_err = (err / std::abs(avg)) * 100.0;
        std::cout << "Real error: " << std::fixed << std::setprecision(2) << rel_err << " %" << std::endl;
    };

    const auto solve_simplex = [&original, print_obj_GOST]() {
        const auto r = SimplexSolver::solve(original, false);
        const auto dual = DualBuilder::build_dual(original);
        const auto r_dual = SimplexSolver::solve(dual, false);
        print_simplex_solution(r);
        print_obj_GOST(r.objective_value, r_dual.objective_value);
    };
    solve_simplex();

    const auto solve_enum = [&original, print_obj_GOST]() {
        const auto r = EnumSolver::solve(original, false);
        const auto dual = DualBuilder::build_dual(original);
        const auto r_dual = EnumSolver::solve(dual, false);
        print_obj_GOST(r.objective_value, r_dual.objective_value);
    };
    solve_enum();

    return 0;
}
