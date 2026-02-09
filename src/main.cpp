#include <iostream>
#include <string>
#include "DualBuilder.hpp"
#include "FormConverter.hpp"
#include "LinearProgram.hpp"

#include "SimplexSolver.hpp"

void print_solution(const std::optional<SimplexSolver::Solution>& res_opt) {
    if (!res_opt) {
        std::cout << "Simplex failed" << std::endl;
        return;
    }
    const auto& res = res_opt.value();

    const auto print_status = [&res]() {
        if (!res.status.empty()) {
            std::cout << "Status: " << res.status;
        }
        std::cout << std::endl;
    };

    const auto print_vector = [](const std::vector<double>& t) {
        std::cout << '(';
        if (!t.empty()) {
            for (int i = 0; i < t.size() - 1; i++) {
                std::cout << t[i] << ", ";
            }
            std::cout << t[t.size() - 1];
        }
        std::cout << ')' << std::endl;
    };

    if (res.is_infeasible) {
        std::cout << "Simplex failed: task is infeasible";
        print_status();
        return;
    }

    if (res.is_unbounded) {
        std::cout << "Simplex failed: task is unbounded";
        print_status();
        return;
    }

    std::cout << "Simplex solution:" << std::endl;
    std::cout << "\tObjective value: " << res.objective_value << std::endl;
    std::cout << "\tx: ";
    print_vector(res.x);
    std::cout << "\tIterations: " << res.iterations << std::endl;
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
        const auto r = SimplexSolver::solve(lp, false);
        print_solution(r);
    }
    {
        const auto r = SimplexSolver::solve(canonical, false);
        print_solution(r);
    }
    {
        const auto r = SimplexSolver::solve(original, false);
        print_solution(r);
    }
    {
        const auto dual = DualBuilder::build_dual(original);
        const auto r = SimplexSolver::solve(dual, false);
        print_solution(r);
    }

    return 0;
}
