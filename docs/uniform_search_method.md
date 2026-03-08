# Uniform Search Algorithm

## Overview
Uniform search (also known as uniform division or grid search) is a technique for finding the minimum of a function by evaluating it at uniformly spaced points across the search interval. The method systematically divides the interval into equal segments and identifies the segment containing the minimum.

## Mathematical Foundation
Given interval [a, b] and n evaluation points, the algorithm evaluates the function at:
- xᵢ = a + i × h, where h = (b - a) / (n - 1), for i = 0, 1, ..., n-1

## Algorithm Steps
1. Initialize interval [a, b] and tolerance ε
2. Determine number of initial points n based on tolerance
3. Calculate step size: h = (b - a) / (n - 1)
4. Evaluate function at all n points: f(xᵢ) for i = 0, 1, ..., n-1
5. Find the point xₘ with minimum function value
6. Refine search around xₘ by creating a new smaller interval
7. Repeat steps 2-6 until interval width ≤ ε
8. Return the point with minimum function value found

## Adaptive Version
A more efficient approach is the adaptive uniform search:
1. Start with interval [a, b] and initial number of points
2. Evaluate function at uniform points
3. Identify the point with minimum value
4. Create a new interval centered at the minimum point with reduced width
5. Repeat until desired precision is achieved

## Convergence Rate
- For n points per iteration, the interval reduction rate depends on the refinement strategy
- Typically slower than golden section search for the same number of function evaluations
- Number of iterations needed depends on the initial interval and required precision

## Advantages
- Simple to implement and understand
- Can detect multiple local minima
- No assumptions about function continuity beyond unimodality
- Parallelizable function evaluations

## Disadvantages
- Requires more function evaluations per iteration than golden section
- Less efficient convergence rate
- Performance highly dependent on number of points chosen