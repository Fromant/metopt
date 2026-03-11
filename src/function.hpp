#ifndef FUNCTION_HPP
#define FUNCTION_HPP

#include <string>

/**
 * @brief Абстрактный базовый класс для математических функций
 */
class Function {
public:
    virtual ~Function() = default;
    /**
     * @brief Вычислить функцию в точке x
     * @param x Точка, в которой вычисляется функция
     * @return Значение функции в x
     */
    virtual double operator()(double x) const = 0;
    
    /**
     * @brief Получить имя/описание функции
     * @return Имя функции
     */
    virtual std::string name() const = 0;
};

/**
 * @brief Реализация целевой функции: f(x) = x^2 - 2x - 2cos(x)
 */
class TargetFunction : public Function {
public:
    double operator()(double x) const override;
    std::string name() const override;
};

#endif // FUNCTION_HPP