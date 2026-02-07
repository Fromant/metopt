#include <iostream>
#include <string>
#include "DualBuilder.hpp"
#include "FormConverter.hpp"
#include "LinearProgram.hpp"

#include "SimplexSolver.hpp"

void print_simplex_result(const std::pair<double, std::vector<double>>& result) {
    std::cout << "Simplex result: " << result.first << "; x=(";
    for (int i = 0; i < result.second.size() - 1; i++) {
        std::cout << result.second[i] << ", ";
    }
    std::cout << result.second[result.second.size() - 1] << ")" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "Linear Programming Forms Converter" << std::endl;
    std::cout << "Converts LP problems to general, symmetric, and canonical forms" << std::endl;
    std::cout << "and constructs corresponding dual problems." << std::endl;

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

    std::cout << "Form of input LP problem: " << original.getForm() << std::endl;

    // Convert to general form (formula 4.1) - this is the canonical "general form"
    LinearProgram general = FormConverter::to_general_form(original);
    general.print("GENERAL FORM (formula 4.1)");
    LinearProgram dual_general = DualBuilder::build_dual(general);
    dual_general.print("DUAL PROBLEM FOR GENERAL FORM");

    // Convert to symmetric form (formula 4.2)
    LinearProgram symmetric = FormConverter::to_symmetric_form(original);
    symmetric.print("SYMMETRIC FORM (formula 4.2)");
    LinearProgram dual_symmetric = DualBuilder::build_dual(symmetric);
    dual_symmetric.print("DUAL PROBLEM FOR SYMMETRIC FORM");

    // Convert to canonical form (formula 4.3)
    LinearProgram canonical = FormConverter::to_canonical_form(original);
    canonical.print("CANONICAL FORM (formula 4.3)");
    LinearProgram dual_canonical = DualBuilder::build_dual(canonical);
    dual_canonical.print("DUAL PROBLEM FOR CANONICAL FORM");

    // Пример задачи: минимизировать x1 + x2 при условии x1 + x2 = 1, x1,x2 >= 0
    LinearProgram lp(true, // минимизация
                     {1.0, 1.0}, // целевая функция
                     {{1.0, 1.0}}, // матрица ограничений
                     {"="}, // типы ограничений
                     {1.0}, // правая часть
                     {">=0", ">=0"} // ограничения на переменные
    );

    {
        const auto r = SimplexSolver::solve(lp);
        print_simplex_result({r.objective_value, r.x});
    }
    {
        const auto r = SimplexSolver::solve(canonical);
        print_simplex_result({r.objective_value, r.x});
    }

    return 0;
}
