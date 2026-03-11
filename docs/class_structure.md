# Структура классов C++ для алгоритмов оптимизации

## Общая архитектура

```
project_root/
├── CMakeLists.txt
├── src/
│   ├── main.cpp                 # Точка входа с командным интерфейсом
│   ├── function.hpp             # Определение целевой функции
│   ├── function.cpp             # Реализация целевой функции
│   ├── optimizer_base.hpp       # Абстрактный базовый класс для оптимизаторов
│   ├── golden_section.hpp       # Объявление оптимизатора методом золотого сечения
│   ├── golden_section.cpp       # Реализация оптимизатора методом золотого сечения
│   ├── uniform_search.hpp       # Объявление оптимизатора равномерного поиска
│   └── uniform_search.cpp       # Реализация оптимизатора равномерного поиска
├── include/
│   └── (публичные заголовки)
├── tests/
│   ├── CMakeLists.txt
│   ├── test_main.cpp            # Основной модуль GoogleTest
│   ├── test_golden_section.cpp  # Тесты для метода золотого сечения
│   ├── test_uniform_search.cpp  # Тесты для метода равномерного поиска
│   └── test_function.cpp        # Тесты для целевой функции
├── docs/                       # Файлы документации
└── plots/                      # Выходной каталог для графиков
```

## Иерархия классов

### 1. Класс Function
```cpp
// function.hpp
class Function {
public:
    virtual ~Function() = default;
    virtual double operator()(double x) const = 0;
    virtual std::string name() const = 0;
};
```

```cpp
// Реализация для f(x) = x² - 2x - 2cos(x)
class TargetFunction : public Function {
public:
    double operator()(double x) const override;
    std::string name() const override;
};
```

### 2. Абстрактный базовый класс оптимизатора
```cpp
// optimizer_base.hpp
struct OptimizationResult {
    double min_x;           // x-значение в минимуме
    double min_value;       // значение функции в минимуме
    int num_evaluations;    // количество вычислений функции
    double final_interval_width; // ширина конечного интервала
};

class OptimizerBase {
protected:
    int evaluation_count;

public:
    OptimizerBase();
    virtual ~OptimizerBase() = default;
    
    virtual OptimizationResult minimize(
        const Function& func,
        double a, 
        double b, 
        double epsilon
    ) = 0;
    
    int get_evaluation_count() const;
    void reset_evaluation_count();
};
```

### 3. Оптимизатор методом золотого сечения
```cpp
// golden_section.hpp
class GoldenSectionOptimizer : public OptimizerBase {
private:
    static constexpr double GOLDEN_RATIO = 0.6180339887498949; // (sqrt(5)-1)/2

public:
    OptimizationResult minimize(
        const Function& func,
        double a, 
        double b, 
        double epsilon
    ) override;
};
```

### 4. Оптимизатор равномерного поиска
```cpp
// uniform_search.hpp
class UniformSearchOptimizer : public OptimizerBase {
private:
    int points_per_iteration; // настраиваемое количество точек на итерацию

public:
    explicit UniformSearchOptimizer(int points = 5);
    
    OptimizationResult minimize(
        const Function& func,
        double a, 
        double b, 
        double epsilon
    ) override;
};
```

## Структура основного приложения
```cpp
// main.cpp
int main(int argc, char* argv[]) {
    // Разбор аргументов командной строки для точности
    // Создание целевой функции
    // Запуск обоих методов оптимизации
    // Сравнение результатов с теоретическими предсказаниями
    // Отображение результатов
    return 0;
}
```

## Структура тестирования
Каждый компонент будет иметь соответствующие модульные тесты с использованием GoogleTest:
- Тестирование реализации целевой функции
- Тестирование алгоритма золотого сечения с известными функциями
- Тестирование алгоритма равномерного поиска с известными функциями
- Тестирование граничных случаев и условий ошибок
- Проверка соответствия количества итераций теоретическим ожиданиям