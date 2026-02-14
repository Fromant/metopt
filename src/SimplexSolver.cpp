#include "SimplexSolver.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include "FormConverter.hpp"

constexpr double TOL = 1e-9;
constexpr int MAX_ITERATIONS = 1000;

// Восстанавливает решение исходной задачи из решения канонической формы
std::vector<double> restore_original_solution(const LinearProgram& original_lp,
                                              const std::vector<double>& canonical_solution) {
    const int n_orig = original_lp.num_variables();
    std::vector<double> original_solution(n_orig, 0.0);

    // Определяем позиции "минус" частей для свободных переменных
    // Пример: если свободные переменные имеют индексы [3, 4] в исходной задаче,
    // то их "минус" части будут находиться по индексам [n_orig + 0, n_orig + 1]
    std::vector<int> free_neg_indices;
    for (int i = 0; i < n_orig; ++i) {
        if (original_lp.var_constraints()[i] == "free") {
            free_neg_indices.push_back(n_orig + static_cast<int>(free_neg_indices.size()));
        }
    }

    // Восстанавливаем значения переменных
    int free_counter = 0;
    for (int i = 0; i < n_orig; ++i) {
        if (original_lp.var_constraints()[i] == "free") {
            // x_free = x+ - x-
            double pos_part = i < canonical_solution.size() ? canonical_solution[i] : 0.0;
            double neg_part =
                (free_counter < free_neg_indices.size() && free_neg_indices[free_counter] < canonical_solution.size())
                ? canonical_solution[free_neg_indices[free_counter]]
                : 0.0;
            original_solution[i] = pos_part - neg_part;
            free_counter++;
        } else {
            // Неотрицательные переменные берём напрямую
            original_solution[i] = (i < canonical_solution.size()) ? canonical_solution[i] : 0.0;
        }
    }

    return original_solution;
}

std::optional<SimplexSolver::Solution> SimplexSolver::solve(const LinearProgram& lp, bool verbose) {
    if (verbose) {
        lp.print("Simplex solver. Original problem:");
    }

    const LinearProgram canonical = FormConverter::to_canonical_form(lp);

    if (verbose && lp.getForm() != LinearProgram::CANONIC) {
        canonical.print("Problem in canonical form: ");
    }

    // PHASE I: Find feasible solution using artificial basis
    auto state_opt = phase1(canonical, verbose);
    if (!state_opt) {
        return std::nullopt;
    }

    auto state = *state_opt;

    // Check feasibility (Theorem 4.1)
    double phase1_obj = 0.0;
    for (int i = 0; i < state.m; ++i) {
        if (state.basis[i] >= state.n) { // Artificial variable in basis
            phase1_obj += state.x_B(i);
        }
    }

    if (std::abs(phase1_obj) > TOL) {
        if (verbose) {
            std::cout << "\n===== INFEASIBLE PROBLEM =====" << std::endl;
            std::cout << "Phase I objective value = " << phase1_obj << " > tolerance (" << TOL << ")" << std::endl;
            std::cout << "Original problem has no feasible solution." << std::endl;
        }
        return Solution{{}, 0.0, false, true, 0, "Infeasible"};
    }

    if (verbose) {
        std::cout << "\n===== REMOVING ARTIFICIAL VARIABLES FROM BASIS =====" << std::endl;
    }
    remove_artificial_vars(state, canonical.num_variables(), verbose);

    // PHASE II: Optimize original objective
    auto result = phase2(state, canonical, verbose);

    result.x = restore_original_solution(lp, result.x);

    // Коррекция знака целевой функции для задач максимизации
    if (!lp.is_minimization()) {
        result.objective_value = -result.objective_value;
    }

    return result;
}

std::optional<SimplexSolver::SimplexState> SimplexSolver::phase1(const LinearProgram& lp, bool verbose) {
    const int m = lp.num_constraints();
    const int n = lp.num_variables();

    Eigen::MatrixXd A(m, n);
    Eigen::VectorXd b(m);
    prepare_rhs_nonnegative(A, b, lp);

    // Extended matrix for Phase I: [A | I] (equation 5.11)
    Eigen::MatrixXd A_ext(m, n + m);
    A_ext.leftCols(n) = A;
    A_ext.rightCols(m) = Eigen::MatrixXd::Identity(m, m);

    // Phase I objective: min sum of artificial variables
    Eigen::VectorXd c_phase1(n + m);
    c_phase1.head(n).setZero();
    c_phase1.tail(m).setOnes();

    // Initial basis: artificial variables (indices n..n+m-1)
    std::vector<int> basis(m);
    for (int i = 0; i < m; ++i)
        basis[i] = n + i;

    // Initial basic solution: x_B = b
    Eigen::VectorXd x_B = b;

    // Initial inverse basis matrix: B = I => B⁻¹ = I
    Eigen::MatrixXd B_inv = Eigen::MatrixXd::Identity(m, m);

    if (verbose) {
        std::cout << "\n===== PHASE I: FINDING FEASIBLE SOLUTION =====" << std::endl;
        std::cout << "Problem dimensions: " << m << " constraints, " << n << " original variables" << std::endl;
        std::cout << "Added " << m << " artificial variables" << std::endl;
        std::cout << "Initial basis: {";
        for (size_t i = 0; i < basis.size(); ++i) {
            std::cout << basis[i] << (i < basis.size() - 1 ? ", " : "");
        }
        std::cout << "}" << std::endl;
    }

    int iter = 0;
    while (iter < MAX_ITERATIONS) {
        // STEP 1: Compute dual variables y = c_B · B⁻¹ (equation 5.2)
        Eigen::VectorXd c_B(m);
        for (int i = 0; i < m; ++i)
            c_B(i) = c_phase1(basis[i]);
        Eigen::VectorXd y = step1_compute_dual_vars(c_B, B_inv);

        // STEP 2: Compute reduced costs d = c - Aᵀ·y (equations 5.3-5.5)
        Eigen::VectorXd d = step2_compute_reduced_costs(c_phase1, y, A_ext);

        // STEP 3: Check optimality condition (4.20)
        auto [optimal, entering] = step3_check_optimality(d, basis, n + m);
        if (optimal) {
            if (verbose) {
                std::cout << "\nPhase I completed in " << iter << " iterations" << std::endl;
                double obj_val = 0.0;
                for (int i = 0; i < m; ++i) {
                    if (basis[i] >= n)
                        obj_val += x_B(i);
                }
                std::cout << "Phase I objective value: " << obj_val << std::endl;
            }
            return SimplexState{A_ext, b, c_phase1, basis, x_B, B_inv, m, n + m};
        }

        // STEP 4: Compute descent direction u = B⁻¹ · a_j (before equation 5.7)
        Eigen::VectorXd a_j = A_ext.col(entering);
        Eigen::VectorXd u = step4_compute_direction(B_inv, a_j);

        // STEP 5: Check unboundedness
        if (step5_check_unboundedness(u)) {
            if (verbose)
                std::cerr << "Error: Phase I is unbounded (original problem infeasible)" << std::endl;
            return std::nullopt;
        }

        // STEP 6: Select leaving variable and step size \theta (equation 5.6)
        auto [leaving_idx, theta] = step6_select_leaving_variable(x_B, u);
        if (leaving_idx == -1) {
            if (verbose)
                std::cerr << "Error: No leaving variable found" << std::endl;
            return std::nullopt;
        }

        bool degenerate = (theta < TOL);
        int leaving_var = basis[leaving_idx];

        // STEP 7: Update solution x_new = x_old - θ·u (equation 5.7)
        x_B = step7_update_solution(x_B, u, theta, leaving_idx);

        // STEP 8: Update inverse basis matrix B⁻¹_new = F · B⁻¹_old (equations 5.8-5.10)
        B_inv = step8_update_basis_inverse(B_inv, u, leaving_idx);

        // Update basis
        basis[leaving_idx] = entering;

        print_iteration(iter, entering, leaving_var, leaving_idx, theta, basis, x_B, d(entering), degenerate, verbose);

        iter++;
    }

    if (verbose)
        std::cerr << "Error: Maximum iterations exceeded in Phase I" << std::endl;
    return std::nullopt;
}

SimplexSolver::Solution SimplexSolver::phase2(SimplexSolver::SimplexState state, const LinearProgram& lp,
                                              bool verbose) {
    // Restore original objective function (only first n variables)
    Eigen::VectorXd c_original(lp.num_variables());
    for (int i = 0; i < lp.num_variables(); ++i) {
        c_original(i) = lp.objective()[i];
    }
    state.c = c_original;
    state.n = lp.num_variables();

    if (verbose) {
        std::cout << "\n===== PHASE II: OPTIMIZING ORIGINAL OBJECTIVE =====" << std::endl;
        std::cout << "Initial basis: {";
        for (size_t i = 0; i < state.basis.size(); ++i) {
            std::cout << state.basis[i] << (i < state.basis.size() - 1 ? ", " : "");
        }
        std::cout << "}" << std::endl;
    }

    int iter = 0;
    int degenerate_count = 0;

    while (iter < MAX_ITERATIONS) {
        // STEP 1: Compute dual variables
        Eigen::VectorXd c_B(state.m);
        for (int i = 0; i < state.m; ++i) {
            if (state.basis[i] >= state.n) {
                c_B(i) = 0.0; // Artificial variables don't affect objective in Phase II
            } else {
                c_B(i) = state.c(state.basis[i]);
            }
        }
        Eigen::VectorXd y = step1_compute_dual_vars(c_B, state.B_inv);

        // STEP 2: Compute reduced costs (only for original variables)
        Eigen::VectorXd d = step2_compute_reduced_costs(state.c, y, state.A.leftCols(state.n));

        // STEP 3: Check optimality
        auto [optimal, entering] = step3_check_optimality(d, state.basis, state.n);
        if (optimal) {
            if (verbose) {
                std::cout << "\n===== OPTIMAL SOLUTION FOUND =====" << std::endl;
                if (degenerate_count > 0) {
                    std::cout << "Degenerate pivots encountered: " << degenerate_count << std::endl;
                }
            }
            break;
        }

        // STEPS 4-8: Same as Phase I
        Eigen::VectorXd a_j = state.A.col(entering);
        Eigen::VectorXd u = step4_compute_direction(state.B_inv, a_j);

        if (step5_check_unboundedness(u)) {
            if (verbose)
                std::cout << "\n===== UNBOUNDED PROBLEM =====" << std::endl;
            return Solution{{}, 0.0, true, false, iter, "Unbounded"};
        }

        auto [leaving_idx, theta] = step6_select_leaving_variable(state.x_B, u);
        if (leaving_idx == -1) {
            if (verbose)
                std::cerr << "Error: No leaving variable found in Phase II" << std::endl;
            return Solution{{}, 0.0, false, true, iter, "Error"};
        }

        bool degenerate = (theta < TOL);
        if (degenerate)
            degenerate_count++;
        int leaving_var = state.basis[leaving_idx];

        state.x_B = step7_update_solution(state.x_B, u, theta, leaving_idx);
        state.B_inv = step8_update_basis_inverse(state.B_inv, u, leaving_idx);
        state.basis[leaving_idx] = entering;

        print_iteration(iter, entering, leaving_var, leaving_idx, theta, state.basis, state.x_B, d(entering),
                        degenerate, verbose);

        iter++;
    }

    // Construct full solution vector (only original variables)
    std::vector<double> solution(lp.num_variables(), 0.0);
    for (int i = 0; i < state.m; ++i) {
        if (state.basis[i] < lp.num_variables()) {
            solution[state.basis[i]] = state.x_B(i);
        }
    }

    // Compute objective value
    double obj_value = 0.0;
    for (int i = 0; i < lp.num_variables(); ++i) {
        obj_value += lp.objective()[i] * solution[i];
    }

    if (verbose) {
        std::cout << "\n===== FINAL SOLUTION =====" << std::endl;
        std::cout << "Objective value: " << std::fixed << std::setprecision(6) << obj_value << std::endl;
        std::cout << "Solution vector: [";
        for (int i = 0; i < lp.num_variables(); ++i) {
            std::cout << solution[i] << (i < lp.num_variables() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        std::cout << "Total iterations: " << iter << std::endl;
    }

    return Solution{solution, obj_value, false, false, iter, "Optimal"};
}

// ==================== STEP 1: DUAL VARIABLES (equation 5.2) ====================
Eigen::VectorXd SimplexSolver::step1_compute_dual_vars(const Eigen::VectorXd& c_B, const Eigen::MatrixXd& B_inv) {
    // Mathematical derivation:
    //   Textbook: y_row = c_B_row · B⁻¹        (row vectors)
    //   In code (column vectors):
    //        y_col = (c_B_row · B⁻¹)^T
    //              = (B⁻¹)^T · c_B_col
    //              = B_inv.transpose() * c_B
    return B_inv.transpose() * c_B;
}

// ==================== STEP 2: REDUCED COSTS (equations 5.3-5.5) ====================
Eigen::VectorXd SimplexSolver::step2_compute_reduced_costs(const Eigen::VectorXd& c, const Eigen::VectorXd& y,
                                                           const Eigen::MatrixXd& A) {
    // d = c - Aᵀ · y  (equation 5.5)
    // Size check:
    //   A: m × n  →  Aᵀ: n × m
    //   y: m × 1
    //   Aᵀ·y: n × 1
    //   c: n × 1  →  d: n × 1 (compatible for subtraction)
    return c - A.transpose() * y;
}

// ==================== STEP 3: OPTIMALITY CHECK (condition 4.20) ====================
std::pair<bool, int> SimplexSolver::step3_check_optimality(const Eigen::VectorXd& d, const std::vector<int>& basis,
                                                           int n_vars) {
    bool optimal = true;
    int entering = -1;
    double min_d = 0.0;

    for (int j = 0; j < n_vars; ++j) {
        if (std::find(basis.begin(), basis.end(), j) != basis.end())
            continue;

        if (d(j) < -TOL) {
            optimal = false;
            if (d(j) < min_d) {
                min_d = d(j);
                entering = j;
            }
        }
    }

    return {optimal, entering};
}

// ==================== STEP 4: DESCENT DIRECTION (before equation 5.7) ====================
Eigen::VectorXd SimplexSolver::step4_compute_direction(const Eigen::MatrixXd& B_inv, const Eigen::VectorXd& a_j) {
    // u = B⁻¹ · a_j
    return B_inv * a_j;
}

// ==================== STEP 5: UNBOUNDEDNESS CHECK ====================
bool SimplexSolver::step5_check_unboundedness(const Eigen::VectorXd& u) {
    for (int i = 0; i < u.size(); ++i) {
        if (u(i) > TOL)
            return false;
    }
    return true;
}

// ==================== STEP 6: LEAVING VARIABLE SELECTION (equation 5.6) ====================
std::pair<int, double> SimplexSolver::step6_select_leaving_variable(const Eigen::VectorXd& x_B,
                                                                    const Eigen::VectorXd& u) {
    double theta = std::numeric_limits<double>::max();
    int leaving_idx = -1;

    for (int i = 0; i < x_B.size(); ++i) {
        if (u(i) > TOL) {
            double ratio = x_B(i) / u(i);
            if (ratio < theta - TOL) {
                theta = ratio;
                leaving_idx = i;
            }
        }
    }

    return {leaving_idx, theta};
}

// ==================== STEP 7: SOLUTION UPDATE (equation 5.7) ====================
Eigen::VectorXd SimplexSolver::step7_update_solution(const Eigen::VectorXd& x_B_old, const Eigen::VectorXd& u,
                                                     double theta, int leaving_idx) {
    Eigen::VectorXd x_B_new = x_B_old - theta * u;
    x_B_new(leaving_idx) = theta; // Entering variable takes value θ at leaving position
    return x_B_new;
}

// ==================== STEP 8: INVERSE BASIS UPDATE (equations 5.8-5.10) ====================
Eigen::MatrixXd SimplexSolver::step8_update_basis_inverse(const Eigen::MatrixXd& B_inv_old, const Eigen::VectorXd& u,
                                                          int leaving_idx) {
    int m = B_inv_old.rows();
    double u_r = u(leaving_idx);

    // Handle near-zero pivot (degenerate case) with numerical safety
    if (std::abs(u_r) < TOL) {
        // In true degenerate cycling, this would require anti-cycling rules
        // For educational purposes, we use a small perturbation
        u_r = (u_r >= 0) ? TOL : -TOL;
    }

    // Construct Frobenius matrix F (equation 5.10)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(m, m);
    for (int i = 0; i < m; ++i) {
        if (i != leaving_idx) {
            F(i, leaving_idx) = -u(i) / u_r;
        } else {
            F(i, leaving_idx) = 1.0 / u_r;
        }
    }

    // Update inverse basis: B⁻¹_new = F · B⁻¹_old (equation 5.8)
    return F * B_inv_old;
}

// ==================== AUXILIARY METHODS ====================
void SimplexSolver::prepare_rhs_nonnegative(Eigen::MatrixXd& A, Eigen::VectorXd& b, const LinearProgram& lp) {
    int m = lp.num_constraints();
    int n = lp.num_variables();

    A.resize(m, n);
    b.resize(m);

    for (int i = 0; i < m; ++i) {
        b(i) = lp.rhs()[i];
        if (b(i) < -TOL) {
            b(i) = -b(i);
            for (int j = 0; j < n; ++j) {
                A(i, j) = -lp.constraints()[i][j];
            }
        } else {
            for (int j = 0; j < n; ++j) {
                A(i, j) = lp.constraints()[i][j];
            }
        }
    }
}

void SimplexSolver::remove_artificial_vars(SimplexSolver::SimplexState& state, int original_n, bool verbose) {
    bool changed = true;
    int attempts = 0;

    while (changed && attempts < MAX_ITERATIONS) {
        changed = false;
        attempts++;

        for (int i = 0; i < state.m; ++i) {
            if (state.basis[i] >= original_n) {
                int replacement = -1;
                for (int j = 0; j < original_n; ++j) {
                    if (std::find(state.basis.begin(), state.basis.end(), j) != state.basis.end())
                        continue;

                    Eigen::VectorXd a_j = state.A.col(j);
                    Eigen::VectorXd u = state.B_inv * a_j;

                    if (std::abs(u(i)) > TOL) {
                        replacement = j;
                        break;
                    }
                }

                if (replacement != -1) {
                    state.basis[i] = replacement;
                    changed = true;

                    if (verbose) {
                        std::cout << "Replaced artificial variable x_" << state.basis[i] << " with original variable x_"
                                  << replacement << std::endl;
                    }
                    break;
                }
            }
        }
    }
}

void SimplexSolver::print_iteration(int iter, int entering, int leaving_var, int leaving_idx, double theta,
                                    const std::vector<int>& basis, const Eigen::VectorXd& x_B, double reduced_cost,
                                    bool degenerate, bool verbose) {
    if (!verbose)
        return;

    std::cout << "\n--- Iteration " << iter << " ---" << std::endl;
    std::cout << "Entering variable: x_" << entering << " (reduced cost = " << reduced_cost << ")" << std::endl;
    std::cout << "Leaving variable: x_" << leaving_var << " (position " << leaving_idx << ")" << std::endl;
    std::cout << "Step size \\theta = " << theta << (degenerate ? " [DEGENERATE]" : "") << std::endl;
    std::cout << "New basis: {";
    for (size_t i = 0; i < basis.size(); ++i) {
        std::cout << basis[i] << (i < basis.size() - 1 ? ", " : "");
    }
    std::cout << "}" << std::endl;
    std::cout << "Basic solution x_B = [";
    for (int i = 0; i < x_B.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6) << x_B(i) << (i < x_B.size() - 1 ? ", " : "");
    }
    std::cout << "]" << std::endl;
}
