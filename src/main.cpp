#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include "function.hpp"
#include "golden_section.hpp"
#include "uniform_search.hpp"


int calculateGoldenSectionIterations(double initial_width, double epsilon) {
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;

    if (initial_width <= epsilon) {
        return 0;
    }

    double ratio = initial_width / epsilon;
    int iterations = static_cast<int>(std::ceil(std::log(ratio) / std::log(phi)));

    return iterations;
}


int calculateUniformSearchIterations(double initial_width, double epsilon, int points_per_iter) {

    
    const double shrink_factor = static_cast<double>(points_per_iter - 1) / 2.0;
    
    if (shrink_factor <= 1.0) {
        return points_per_iter + 1;
    }
    
    double ratio = initial_width / epsilon;
    int iterations = static_cast<int>(std::ceil(
        std::log(ratio) / std::log(shrink_factor)
    ));
    
    return iterations * points_per_iter + 1;
}


int main() {
    // Создание целевой функции
    TargetFunction func;

    // Определение интервала
    double a = 0.5;
    double b = 1.0;

    // Определение уровней точности
    std::vector<double> epsilons = {0.1, 0.01, 0.001};

    std::cout << "Оптимизация " << func.name() << " на [" << a << ", " << b << "]" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    for (double epsilon : epsilons) {
        std::cout << "\nТочность: " << epsilon << std::endl;
        std::cout << std::string(40, '-') << std::endl;

        // Метод золотого сечения
        GoldenSectionOptimizer gs_optimizer;
        auto gs_result = gs_optimizer.minimize(func, a, b, epsilon);

        // Расчёт теоретических итераций для метода золотого сечения
        int gs_theoretical = calculateGoldenSectionIterations(b - a, epsilon);

        std::cout << "Метод золотого сечения:" << std::endl;
        std::cout << "  x минимума: " << std::fixed << std::setprecision(6) << gs_result.min_x << std::endl;
        std::cout << "  Значение минимума: " << std::fixed << std::setprecision(6) << gs_result.min_value << std::endl;
        std::cout << "  Вычислений функции: " << gs_result.num_evaluations << std::endl;
        std::cout << "  Ширина конечного интервала: " << gs_result.final_interval_width << std::endl;
        std::cout << "  Теоретически необходимое количество итераций: " << gs_theoretical << std::endl;

        // Метод равномерного поиска
        UniformSearchOptimizer us_optimizer(5); // Использование 5 точек за итерацию
        auto us_result = us_optimizer.minimize(func, a, b, epsilon);

        // Расчёт теоретических итераций для метода равномерного поиска
        int us_theoretical = calculateUniformSearchIterations(b - a, epsilon, 5);

        std::cout << "\nМетод равномерного поиска (5 точек за итерацию):" << std::endl;
        std::cout << "  x минимума: " << std::fixed << std::setprecision(6) << us_result.min_x << std::endl;
        std::cout << "  Значение минимума: " << std::fixed << std::setprecision(6) << us_result.min_value << std::endl;
        std::cout << "  Вычислений функции: " << us_result.num_evaluations << std::endl;
        std::cout << "  Ширина конечного интервала: " << us_result.final_interval_width << std::endl;
        std::cout << "  Теоретически необходимое количество итераций: " << us_theoretical << std::endl;
    }

    return 0;
}
