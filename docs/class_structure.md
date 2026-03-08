# C++ Class Structure for Optimization Algorithms

## Overall Architecture

```
project_root/
├── CMakeLists.txt
├── src/
│   ├── main.cpp                 # Entry point with command-line interface
│   ├── function.hpp             # Target function definition
│   ├── function.cpp             # Target function implementation
│   ├── optimizer_base.hpp       # Abstract base class for optimizers
│   ├── golden_section.hpp       # Golden section optimizer declaration
│   ├── golden_section.cpp       # Golden section optimizer implementation
│   ├── uniform_search.hpp       # Uniform search optimizer declaration
│   └── uniform_search.cpp       # Uniform search optimizer implementation
├── include/
│   └── (public headers)
├── tests/
│   ├── CMakeLists.txt
│   ├── test_main.cpp            # GoogleTest main
│   ├── test_golden_section.cpp  # Tests for golden section method
│   ├── test_uniform_search.cpp  # Tests for uniform search method
│   └── test_function.cpp        # Tests for target function
├── docs/                       # Documentation files
└── plots/                      # Output directory for plots
```

## Class Hierarchy

### 1. Function Class
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
// Implementation for f(x) = x² - 2x - 2cos(x)
class TargetFunction : public Function {
public:
    double operator()(double x) const override;
    std::string name() const override;
};
```

### 2. Abstract Optimizer Base Class
```cpp
// optimizer_base.hpp
struct OptimizationResult {
    double min_x;           // x-value at minimum
    double min_value;       // function value at minimum
    int num_evaluations;    // number of function evaluations
    double final_interval_width; // width of final interval
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

### 3. Golden Section Optimizer
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

### 4. Uniform Search Optimizer
```cpp
// uniform_search.hpp
class UniformSearchOptimizer : public OptimizerBase {
private:
    int points_per_iteration; // configurable number of points per iteration

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

## Main Application Structure
```cpp
// main.cpp
int main(int argc, char* argv[]) {
    // Parse command line arguments for precision
    // Create target function
    // Run both optimization methods
    // Compare results with theoretical predictions
    // Display results
    return 0;
}
```

## Testing Structure
Each component will have corresponding unit tests using GoogleTest:
- Test the target function implementation
- Test the golden section algorithm with known functions
- Test the uniform search algorithm with known functions
- Test edge cases and error conditions
- Verify iteration counts match theoretical expectations