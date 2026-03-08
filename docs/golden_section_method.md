# Golden Section Search Algorithm

## Overview
The golden section search is a technique for finding the extremum (minimum or maximum) of a unimodal function by successively narrowing the range of values inside which the extremum is known to exist. The algorithm maintains the function values for three points whose distances form a golden ratio.

## Mathematical Foundation
The golden ratio φ = (1 + √5)/2 ≈ 1.618, and we use ρ = φ - 1 = (√5 - 1)/2 ≈ 0.618.

For minimization on interval [a, b]:
- Two interior points are selected: 
  - x₁ = a + (1-ρ)(b-a)
  - x₂ = a + ρ(b-a)
- Since ρ² = 1 - ρ, we have x₁ = a + ρ²(b-a) and x₂ = a + ρ(b-a)

## Algorithm Steps
1. Initialize interval [a₀, b₀] and tolerance ε
2. Calculate initial points:
   - d = ρ(b₀ - a₀)
   - x₁ = a₀ + d
   - x₂ = b₀ - d
3. Evaluate f(x₁) and f(x₂)
4. Iterate until |bₙ - aₙ| ≤ ε:
   - If f(x₁) ≤ f(x₂):
     - Set [aₙ₊₁, bₙ₊₁] = [aₙ, x₂]
     - New x₂ = x₁, calculate new x₁
   - Else:
     - Set [aₙ₊₁, bₙ₊₁] = [x₁, bₙ]
     - New x₁ = x₂, calculate new x₂
5. Return midpoint of final interval as estimate

## Convergence Rate
The interval width decreases by factor ρ each iteration:
- |bₙ - aₙ| = ρⁿ|b₀ - a₀|
- Number of iterations needed: n ≥ log(ε/|b₀ - a₀|) / log(ρ)

## Advantages
- No derivative information required
- Guaranteed convergence for unimodal functions
- Efficient reduction of search interval
- Equal functional evaluations per iteration (2 initial, then 1 per iteration)

## Disadvantages
- Requires function to be unimodal
- Slower than Newton-type methods for smooth functions