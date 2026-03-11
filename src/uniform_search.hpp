#ifndef UNIFORM_SEARCH_HPP
#define UNIFORM_SEARCH_HPP

#include "optimizer_base.hpp"

/**
 * @brief Оптимизатор, использующий метод равномерного поиска
 */
class UniformSearchOptimizer : public OptimizerBase {
private:
    int points_per_iteration;  ///< Количество точек, вычисляемых за итерацию

public:
    /**
     * @brief Конструктор
     * @param points Количество точек, используемых за итерацию (по умолчанию: 5)
     */
    explicit UniformSearchOptimizer(int points = 5);

    /**
     * @brief Минимизировать данную функцию методом равномерного поиска
     * @param func Функция для минимизации
     * @param a Левая граница интервала
     * @param b Правая граница интервала
     * @param epsilon Требуемая точность
     * @return OptimizationResult, содержащий результаты
     */
    OptimizationResult minimize(
        const Function& func,
        double a,
        double b,
        double epsilon
    ) override;
};

#endif // UNIFORM_SEARCH_HPP