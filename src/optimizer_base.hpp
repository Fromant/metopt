#ifndef OPTIMIZER_BASE_HPP
#define OPTIMIZER_BASE_HPP

#include "function.hpp"

/**
 * @brief Структура для хранения результатов оптимизации
 */
struct OptimizationResult {
    double min_x;                ///< x-значение в точке минимума
    double min_value;            ///< значение функции в точке минимума
    int num_evaluations;         ///< количество вычислений функции
    double final_interval_width; ///< ширина конечного интервала
};

/**
 * @brief Абстрактный базовый класс для алгоритмов оптимизации
 */
class OptimizerBase {
protected:
    int evaluation_count;  ///< Счётчик вычислений функции

public:
    OptimizerBase() : evaluation_count(0) {}
    virtual ~OptimizerBase() = default;

    /**
     * @brief Минимизировать данную функцию на указанном интервале
     * @param func Функция для минимизации
     * @param a Левая граница интервала
     * @param b Правая граница интервала
     * @param epsilon Требуемая точность
     * @return OptimizationResult, содержащий результаты
     */
    virtual OptimizationResult minimize(
        const Function& func,
        double a,
        double b,
        double epsilon
    ) = 0;

    /**
     * @brief Получить количество выполненных вычислений функции
     * @return Количество вычислений функции
     */
    int get_evaluation_count() const { return evaluation_count; }

    /**
     * @brief Сбросить счётчик вычислений
     */
    void reset_evaluation_count() { evaluation_count = 0; }

protected:
    /**
     * @brief Увеличить счётчик вычислений
     */
    void increment_evaluation_count() { evaluation_count++; }
};

#endif // OPTIMIZER_BASE_HPP