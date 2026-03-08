# One-Dimensional Optimization Methods

This project implements and compares two classical methods for one-dimensional optimization: the golden section search and uniform search methods. The goal is to find the minimum of a unimodal function within a given interval.

## Implemented Methods

### 1. Golden Section Search
- Based on the golden ratio (φ ≈ 1.618) for optimal interval reduction
- Guarantees convergence for unimodal functions
- Requires 2 initial function evaluations, then 1 per iteration
- Reduces search interval by factor ρ ≈ 0.618 each iteration

### 2. Uniform Search
- Evaluates the function at uniformly spaced points in the interval
- Selects the subinterval containing the minimum point
- Configurable number of points per iteration (default: 5)
- More robust but generally requires more function evaluations

## Target Function

The optimization problem addresses the function:
```
f(x) = x² - 2x - 2cos(x)
```
on the interval [0.5, 1.0].

This function is unimodal in the specified interval, making it suitable for both optimization methods.

## Project Structure

```
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── docs/                       # Documentation
│   ├── golden_section_method.md # Golden section algorithm details
│   ├── uniform_search_method.md # Uniform search algorithm details
│   ├── precision_formula.md     # Theoretical formulas
│   ├── class_structure.md       # C++ class design
│   └── method_comparison.md     # Method comparison analysis
├── src/                        # Source code
│   ├── main.cpp               # Main application
│   ├── function.hpp/cpp       # Target function definition
│   ├── optimizer_base.hpp     # Base optimizer class
│   ├── golden_section.hpp/cpp # Golden section implementation
│   └── uniform_search.hpp/cpp # Uniform search implementation
├── tests/                      # Unit tests
│   ├── test_main.cpp          # Test runner
│   ├── test_function.cpp      # Function tests
│   ├── test_golden_section.cpp # Golden section tests
│   └── test_uniform_search.cpp # Uniform search tests
└── plots/                      # Plotting utilities
    ├── generate_plot_data.cpp # Data generation for plots
    └── plot_function.py       # Visualization script
```

## Building and Running

### Prerequisites
- C++17 compatible compiler
- CMake 3.10 or higher
- Python 3 (for visualization)
- Matplotlib (for visualization)

### Build Instructions
```bash
mkdir build
cd build
cmake ..
make
```

### Running the Application
```bash
./metopt
```

### Running Tests
```bash
./tests
```

### Generating Plots
```bash
# Generate plot data
./plot_generator

# Create visualization
python plots/plot_function.py
```

## Results and Analysis

The application outputs results for three precision levels: 0.1, 0.01, and 0.001.

### Theoretical Predictions

For golden section search:
- Number of iterations needed: n ≥ ln(|b₀ - a₀| / ε) / ln(φ)
- Where φ ≈ 1.618 is the golden ratio

For uniform search (with k points per iteration):
- Number of iterations needed: n ≥ ln(|b₀ - a₀| / ε) / ln(k-1)

### Performance Comparison

The implementation tracks function evaluations to compare actual performance with theoretical predictions. Key metrics include:
- Number of function evaluations
- Final interval width
- Computed minimum value and location
- Comparison with theoretical iteration counts

## Key Findings

1. **Golden Section Search** typically requires fewer function evaluations for high precision requirements
2. **Uniform Search** may be more robust for functions with noise or irregularities
3. Both methods successfully find the minimum of the target function in the specified interval
4. The function f(x) = x² - 2x - 2cos(x) is confirmed to be unimodal in [0.5, 1.0]

## When to Use Each Method

### Golden Section Search is Preferred When:
- Function evaluations are expensive
- High precision is required
- The function is known to be unimodal
- Maximum efficiency in terms of function evaluations is needed

### Uniform Search is Preferred When:
- Function evaluations are relatively cheap
- The function may have noise or irregularities
- Parallel computation is available
- Exploring function behavior across the interval is important

## Mathematical Analysis

The project includes theoretical analysis of the relationship between required precision and number of iterations for both methods, with formulas derived and implemented for comparison with experimental results.

## Visualization

The project includes tools to visualize the target function and demonstrate its unimodal property in the optimization interval, confirming the applicability of both methods.
