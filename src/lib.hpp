#pragma once

#include <Eigen/Dense>
#include <cassert>
#include <random>
#include <vector>

#include "LinearProgram.hpp"


// Восстановление решения исходной задачи из канонической формы
inline std::vector<double> restore_original_solution(const LinearProgram& original_lp,
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
            double pos_part = (i < static_cast<int>(canonical_solution.size())) ? canonical_solution[i] : 0.0;
            double neg_part = 0.0;

            if (free_counter < static_cast<int>(free_neg_indices.size()) &&
                free_neg_indices[free_counter] < static_cast<int>(canonical_solution.size())) {
                neg_part = canonical_solution[free_neg_indices[free_counter]];
            }

            original_solution[i] = pos_part - neg_part;
            free_counter++;
        } else {
            original_solution[i] = (i < static_cast<int>(canonical_solution.size())) ? canonical_solution[i] : 0.0;
        }
    }

    return original_solution;
}

template <typename T>
void print_vector(const std::vector<T>& v) {
    std::cout << "(";
    if (!v.empty()) {
        for (int i = 0; i < v.size() - 1; i++) {
            std::cout << v[i] << ", ";
        }
        std::cout << v[v.size() - 1];
    }
    std::cout << ")" << std::endl;
}

inline Eigen::VectorXd std_to_eigen(const std::vector<double>& v) {
    Eigen::VectorXd result;
    result.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i];
    }
    return result;
}

inline Eigen::VectorXd std_to_eigen(const std::vector<double>& v, const std::function<double(double)>& transform) {
    Eigen::VectorXd result;
    result.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = transform(v[i]);
    }
    return result;
}

inline Eigen::MatrixXd std_to_eigen(const std::vector<std::vector<double>>& m,
                                    const std::function<double(double)>& transform) {
    Eigen::MatrixXd result;
    result.resize(m.size(), m[0].size());
    for (size_t i = 0; i < m.size(); ++i) {
        for (size_t j = 0; j < m[i].size(); ++j) {
            result(i, j) = transform(m[i][j]);
        }
    }
    return result;
}

inline Eigen::MatrixXd std_to_eigen(const std::vector<std::vector<double>>& m) {
    Eigen::MatrixXd result;
    result.resize(m.size(), m[0].size());
    for (size_t i = 0; i < m.size(); ++i) {
        for (size_t j = 0; j < m[i].size(); ++j) {
            result(i, j) = m[i][j];
        }
    }
    return result;
}

inline void print_val_with_err(const double val1, const double val2) {
    double err = std::abs(val1 - val2) / 2;
    double avg = (val1 + val2) / 2;

    const double exponent = std::floor(std::log10(err));
    // Разряд последней значащей цифры погрешности: -exponent
    const auto precision = std::clamp(-exponent, 4.0, 10.0);
    const double factor = std::pow(10.0, -precision - 1); // Сохраняем 2 значащие цифры
    err = std::round(err / factor) * factor;
    avg = std::round(avg / factor) * factor;
    std::cout << std::fixed << std::setprecision(precision) << avg << " +- " << err << std::endl;
    const double rel_err = (err / std::abs(avg)) * 100.0;
    std::cout << "Relative error: " << std::fixed << std::setprecision(2) << rel_err << " %" << std::endl;
}
