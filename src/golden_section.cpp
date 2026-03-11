#include "golden_section.hpp"
#include <cmath>

OptimizationResult GoldenSectionOptimizer::minimize(
    const Function& func,
    double a,
    double b,
    double epsilon
) {
    reset_evaluation_count();

    double x1, x2;
    double f1, f2;

    // Инициализация интервала
    double current_a = a;
    double current_b = b;

    // Вычисление начальных внутренних точек
    double d = GOLDEN_RATIO * (current_b - current_a);
    x1 = current_a + (1.0 - GOLDEN_RATIO) * (current_b - current_a);
    x2 = current_a + GOLDEN_RATIO * (current_b - current_a);

    // Вычисление функции в начальных точках
    f1 = func(x1);
    increment_evaluation_count();
    f2 = func(x2);
    increment_evaluation_count();

    // Итерации до тех пор, пока интервал не станет меньше epsilon
    while (std::abs(current_b - current_a) > epsilon) {
        if (f1 <= f2) {
            // Минимум в [a, x2], поэтому обновляем b = x2 и x2 = x1
            current_b = x2;
            x2 = x1;
            f2 = f1;

            // Вычисление новой точки x1
            x1 = current_a + (1.0 - GOLDEN_RATIO) * (current_b - current_a);
            f1 = func(x1);
            increment_evaluation_count();
        } else {
            // Минимум в [x1, b], поэтому обновляем a = x1 и x1 = x2
            current_a = x1;
            x1 = x2;
            f1 = f2;

            // Вычисление новой точки x2
            x2 = current_a + GOLDEN_RATIO * (current_b - current_a);
            f2 = func(x2);
            increment_evaluation_count();
        }
    }

    // Возвращаем середину конечного интервала как точку минимума
    double min_x = (current_a + current_b) / 2.0;
    double min_value = func(min_x);
    increment_evaluation_count(); // Учитываем финальное вычисление

    OptimizationResult result;
    result.min_x = min_x;
    result.min_value = min_value;
    result.num_evaluations = get_evaluation_count();
    result.final_interval_width = std::abs(current_b - current_a);

    return result;
}