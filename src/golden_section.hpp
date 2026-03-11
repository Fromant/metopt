#ifndef GOLDEN_SECTION_HPP
#define GOLDEN_SECTION_HPP

#include "optimizer_base.hpp"

/**
 * @brief Оптимизатор, использующий метод золотого сечения
 */
class GoldenSectionOptimizer : public OptimizerBase {
private:
    static constexpr double GOLDEN_RATIO = 0.6180339887498949; // (sqrt(5)-1)/2
    static constexpr double INV_GOLDEN_RATIO = 1.0 - GOLDEN_RATIO; // 1 - (sqrt(5)-1)/2 = (3-sqrt(5))/2

public:
    /**
     * @brief Минимизировать данную функцию методом золотого сечения
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

#endif // GOLDEN_SECTION_HPP