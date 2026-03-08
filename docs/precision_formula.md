# Formula Connecting Number of Iterations to Required Precision

## Golden Section Search

### Theory
In the golden section search, the interval is reduced by a constant factor in each iteration.
The golden ratio is φ = (1 + √5)/2 ≈ 1.618
We use ρ = (φ - 1) = (√5 - 1)/2 ≈ 0.618

After each iteration, the interval size is multiplied by ρ.
Starting with interval [a₀, b₀], after n iterations the interval size is:
|bₙ - aₙ| = ρⁿ|b₀ - a₀|

### Formula for Required Iterations
To achieve precision ε (i.e., |bₙ - aₙ| ≤ ε), we need:
ρⁿ|b₀ - a₀| ≤ ε

Taking logarithms:
n × ln(ρ) ≤ ln(ε/|b₀ - a₀|)
Since ln(ρ) < 0, dividing by ln(ρ) flips the inequality:
n ≥ ln(ε/|b₀ - a₀|) / ln(ρ)

Since ρ = (√5 - 1)/2 ≈ 0.618, ln(ρ) ≈ -0.4812
So: n ≥ ln(ε/|b₀ - a₀|) / (-0.4812) = -ln(ε/|b₀ - a₀|) / 0.4812

### Final Formula
n ≥ ln(|b₀ - a₀| / ε) / ln(1/ρ) = ln(|b₀ - a₀| / ε) / ln((√5 + 1)/2)

For our function f(x) = x² - 2x - 2cos(x) on [0.5, 1]:
- |b₀ - a₀| = 0.5
- For ε = 0.1: n ≥ ln(0.5/0.1) / ln(1.618) ≈ ln(5) / 0.481 ≈ 3.32
- For ε = 0.01: n ≥ ln(0.5/0.01) / ln(1.618) ≈ ln(50) / 0.481 ≈ 8.19  
- For ε = 0.001: n ≥ ln(0.5/0.001) / ln(1.618) ≈ ln(500) / 0.481 ≈ 13.07

## Uniform Search

### Theory
For uniform search with k points per iteration, the interval is typically reduced to a fraction of the previous interval based on the distribution of points around the minimum.

If we use k points per iteration and focus on a subinterval that is 2/(k-1) of the original interval around the minimum point, then after n iterations:
|bₙ - aₙ| = (2/(k-1))ⁿ|b₀ - a₀|

However, a simpler approach is to consider that with k points, we divide the interval into k-1 segments, so the resulting uncertainty is approximately |b - a|/(k-1).

For a fixed number of total function evaluations N, if we split into rounds:
- With k points per round, we need approximately log_{k-1}(|b₀ - a₀|/ε) rounds
- Total function evaluations: N ≈ k + (rounds - 1) * k (for first round we evaluate k points, then k-2 for subsequent rounds since we reuse 2 points)

### Practical Formula
For uniform search with fixed number of points k per iteration:
n ≥ log_{k-1}(|b₀ - a₀| / ε)

For our implementation, we'll use k=5 points per iteration:
n ≥ log₄(|b₀ - a₀| / ε) = ln(|b₀ - a₀| / ε) / ln(4)

For our function f(x) = x² - 2x - 2cos(x) on [0.5, 1] with k=5:
- |b₀ - a₀| = 0.5
- For ε = 0.1: n ≥ ln(0.5/0.1) / ln(4) ≈ ln(5) / 1.386 ≈ 1.16
- For ε = 0.01: n ≥ ln(0.5/0.01) / ln(4) ≈ ln(50) / 1.386 ≈ 2.86
- For ε = 0.001: n ≥ ln(0.5/0.001) / ln(4) ≈ ln(500) / 1.386 ≈ 4.57

## Summary of Formulas

### Golden Section Search:
n_theoretical = ⌈ln(|b₀ - a₀| / ε) / ln((√5 + 1)/2)⌉

### Uniform Search (with k points per iteration):
n_theoretical = ⌈ln(|b₀ - a₀| / ε) / ln(k-1)⌉

These formulas will allow us to compare the theoretical number of iterations needed with the actual number of function evaluations performed by our implementations.