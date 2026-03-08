# Comparison of Golden Section Search and Uniform Search Methods

## Computational Efficiency Analysis

### Golden Section Search

#### Advantages:
1. **Optimal reduction rate**: Each iteration reduces the search interval by factor ρ ≈ 0.618
2. **Efficient function evaluations**: After the initial 2 evaluations, only 1 new evaluation per iteration
3. **Predictable behavior**: Consistent interval reduction rate regardless of function shape
4. **Mathematical optimality**: Proven to be optimal for reducing interval size with minimal function evaluations

#### Disadvantages:
1. **Requires unimodal function**: Will not work correctly if function has multiple local minima in the interval
2. **Fixed reduction rate**: Cannot take advantage of favorable function shapes
3. **Sensitive to numerical precision**: May have issues with very high precision requirements due to floating-point limitations

#### Best Use Cases:
- When function evaluations are expensive
- When guaranteed convergence is required
- When the function is known to be unimodal
- When seeking maximum efficiency in terms of function evaluations

### Uniform Search

#### Advantages:
1. **Robustness**: Less sensitive to function irregularities
2. **Multiple sampling**: Provides insight into function behavior across the interval
3. **Parallelizable**: Function evaluations at different points can be done in parallel
4. **Flexibility**: Can adapt to different function behaviors by adjusting number of points

#### Disadvantages:
1. **Higher function evaluations**: Requires k evaluations per iteration (where k is number of points)
2. **Slower convergence**: Generally requires more iterations than golden section
3. **Parameter dependency**: Performance heavily depends on choice of number of points per iteration

#### Best Use Cases:
- When function evaluations are relatively cheap
- When function may have noise or irregularities
- When parallel computation is available
- When exploring function behavior is important

## Theoretical Comparison for f(x) = x² - 2x - 2cos(x)

### Function Properties:
- Domain: [0.5, 1.0]
- The function f(x) = x² - 2x - 2cos(x) is unimodal in this interval
- Derivative: f'(x) = 2x - 2 + 2sin(x)
- Second derivative: f''(x) = 2 + 2cos(x) ≥ 0 (since cos(x) ≥ -1), confirming convexity/local unimodality

### Expected Performance:

For precision ε:
- Golden Section: n ≥ ln((b-a)/ε) / ln(φ) ≈ ln(0.5/ε) / 0.4812
- Uniform Search (k points): n ≥ ln((b-a)/ε) / ln(k-1)

For our test precisions:
- ε = 0.1: GS needs ~3.32 iterations, US(5) needs ~1.16 iterations (but 5× more evals per iter)
- ε = 0.01: GS needs ~8.19 iterations, US(5) needs ~2.86 iterations
- ε = 0.001: GS needs ~13.07 iterations, US(5) needs ~4.57 iterations

### Practical Considerations:

1. **For high precision requirements**: Golden section is typically more efficient
2. **For rough approximations**: Uniform search might be competitive
3. **For noisy functions**: Uniform search provides better robustness
4. **For expensive function evaluations**: Golden section is preferred

## Experimental Results Expected:

Based on the theoretical analysis, we expect:
- Golden section to require fewer total function evaluations
- Uniform search to potentially converge in fewer iterations but with higher cost per iteration
- Both methods to successfully find the minimum in the specified interval
- The actual performance difference to depend on the specific function characteristics

The implementation includes counters for function evaluations to validate these theoretical expectations.