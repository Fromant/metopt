import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar


def target_function(x):
    """
    The target function: f(x) = x^2 - 2x - 2cos(x)
    """
    return x**2 - 2*x - 2*np.cos(x)


def golden_section_search(func, a, b, epsilon, return_history=False):
    """
    Golden section search implementation
    """
    phi = (1 + np.sqrt(5)) / 2  # Golden ratio
    resphi = 2 - phi  # 1/phi = phi - 1
    
    # Initial points
    x1 = a + resphi * (b - a)
    x2 = b - resphi * (b - a)
    f1 = func(x1)
    f2 = func(x2)
    
    eval_count = 2  # Initial function evaluations
    
    if return_history:
        history = [(a, b, (a+b)/2, abs(b-a))]
    else:
        history = []
    
    while abs(b - a) > epsilon:
        if f1 < f2:
            b = x2
            x2 = x1
            f2 = f1
            x1 = a + resphi * (b - a)
            f1 = func(x1)
        else:
            a = x1
            x1 = x2
            f1 = f2
            x2 = b - resphi * (b - a)
            f2 = func(x2)
        
        eval_count += 1
        
        if return_history:
            history.append((a, b, (a+b)/2, abs(b-a)))
    
    min_x = (a + b) / 2
    min_val = func(min_x)
    
    if return_history:
        return min_x, min_val, eval_count, history
    else:
        return min_x, min_val, eval_count


def uniform_search(func, a, b, epsilon, points_per_iter=5, return_history=False):
    """
    Uniform search implementation
    """
    eval_count = 0
    current_a, current_b = a, b
    
    if return_history:
        history = [(current_a, current_b, (current_a+current_b)/2, abs(current_b-current_a))]
    else:
        history = []
    
    while abs(current_b - current_a) > epsilon:
        # Generate evenly spaced points
        x_points = np.linspace(current_a, current_b, points_per_iter)
        f_values = [func(x) for x in x_points]
        eval_count += len(f_values)
        
        # Find the point with minimum value
        min_idx = np.argmin(f_values)
        
        # Determine new interval around the minimum
        if min_idx == 0:  # Minimum at left edge
            current_b = x_points[1]
        elif min_idx == len(x_points) - 1:  # Minimum at right edge
            current_a = x_points[-2]
        else:  # Minimum in middle
            current_a = x_points[min_idx - 1]
            current_b = x_points[min_idx + 1]
        
        if return_history:
            history.append((current_a, current_b, (current_a+current_b)/2, abs(current_b-current_a)))
    
    min_x = (current_a + current_b) / 2
    min_val = func(min_x)
    
    if return_history:
        return min_x, min_val, eval_count, history
    else:
        return min_x, min_val, eval_count


def plot_target_function():
    """
    Plot the target function to demonstrate its unimodality
    """
    x = np.linspace(0, 2, 1000)
    y = target_function(x)
    
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, 'b-', linewidth=2, label=r'$f(x) = x^2 - 2x - 2\cos(x)$')
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('Target Function: Unimodal Analysis')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Find and mark the minimum
    result = minimize_scalar(target_function, bounds=(0, 2), method='bounded')
    plt.plot(result.x, result.fun, 'ro', markersize=8, label=f'Minimum at x={result.x:.4f}')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('plots/target_function.png', dpi=300, bbox_inches='tight')
    plt.show()


def compare_methods_at_different_precisions():
    """
    Compare both methods at different precision levels (0.1, 0.01, 0.001)
    """
    a, b = 0.5, 1.0
    precisions = [0.1, 0.01, 0.001]
    
    print("Comparison of Methods at Different Precisions")
    print("="*60)
    print(f"{'Precision':<12} {'Method':<15} {'Min X':<12} {'Min Value':<15} {'Evaluations':<12}")
    print("-"*60)
    
    for eps in precisions:
        # Golden section
        gs_min_x, gs_min_val, gs_evals = golden_section_search(target_function, a, b, eps)
        print(f"{eps:<12} {'Golden Section':<15} {gs_min_x:<12.6f} {gs_min_val:<15.6f} {gs_evals:<12}")
        
        # Uniform search
        us_min_x, us_min_val, us_evals = uniform_search(target_function, a, b, eps)
        print(f"{eps:<12} {'Uniform Search':<15} {us_min_x:<12.6f} {us_min_val:<15.6f} {us_evals:<12}")
        print("-"*60)


def plot_convergence_history():
    """
    Plot the convergence history of both methods
    """
    a, b = 0.5, 1.0
    epsilon = 0.001
    
    # Get histories
    _, _, _, gs_history = golden_section_search(target_function, a, b, epsilon, return_history=True)
    _, _, _, us_history = uniform_search(target_function, a, b, epsilon, return_history=True)
    
    # Extract data
    gs_iterations = list(range(len(gs_history)))
    gs_intervals = [h[3] for h in gs_history]  # Interval widths
    gs_midpoints = [h[2] for h in gs_history]  # Midpoints
    
    us_iterations = list(range(len(us_history)))
    us_intervals = [h[3] for h in us_history]  # Interval widths
    us_midpoints = [h[2] for h in us_history]  # Midpoints
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot interval width vs iterations
    ax1.semilogy(gs_iterations, gs_intervals, 'b-o', label='Golden Section', markersize=4)
    ax1.semilogy(us_iterations, us_intervals, 'r-s', label='Uniform Search', markersize=4)
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Interval Width')
    ax1.set_title('Convergence: Interval Width vs Iterations')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot midpoint convergence
    ax2.plot(gs_iterations, gs_midpoints, 'b-o', label='Golden Section', markersize=4)
    ax2.plot(us_iterations, us_midpoints, 'r-s', label='Uniform Search', markersize=4)
    ax2.axhline(y=target_function(minimize_scalar(target_function, bounds=(0.5, 1.0), method='bounded').x), 
                color='g', linestyle='--', label='True Minimum')
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Midpoint Value')
    ax2.set_title('Convergence: Midpoint vs Iterations')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('plots/convergence_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def theoretical_vs_actual_evaluations():
    """
    Compare theoretical vs actual function evaluations
    """
    a, b = 0.5, 1.0  # Interval width = 0.5
    precisions = [0.1, 0.01, 0.001]
    
    print("\nTheoretical vs Actual Function Evaluations")
    print("="*80)
    print(f"{'Precision':<10} {'Method':<15} {'Theoretical':<12} {'Actual':<10} {'Difference':<12} {'Efficiency'}")
    print("-"*80)
    
    for eps in precisions:
        # Golden Section theoretical calculation
        # n >= ln(|b0 - a0| / ε) / ln(φ) where φ ≈ 1.618
        golden_ratio = (1 + np.sqrt(5)) / 2
        theoretical_gs = np.ceil(np.log((b - a) / eps) / np.log(golden_ratio))
        # Add 2 for initial evaluations
        theoretical_gs += 2
        
        # Actual evaluations
        _, _, actual_gs = golden_section_search(target_function, a, b, eps)
        
        print(f"{eps:<10} {'Golden Section':<15} {int(theoretical_gs):<12} {actual_gs:<10} {actual_gs-int(theoretical_gs):<12} {'+' if actual_gs <= theoretical_gs else '-'}")
        
        # Uniform Search theoretical calculation
        # For uniform search with k points, after n iterations: |bn - an| = (2/(k-1))^n * |b0 - a0|
        # So we need: (2/(k-1))^n * |b0 - a0| <= ε
        # n >= ln(ε/|b0 - a0|) / ln(2/(k-1))
        k = 5  # points per iteration
        if (2/(k-1)) < 1:  # This is the case for k > 3
            theoretical_us = np.ceil(np.log(eps/(b-a)) / np.log(2/(k-1)))
        else:
            theoretical_us = 1  # Shouldn't happen for k > 3
        
        # Each iteration uses k evaluations
        theoretical_total_us = theoretical_us * k
        
        # Actual evaluations
        _, _, actual_us = uniform_search(target_function, a, b, eps, points_per_iter=5)
        
        print(f"{eps:<10} {'Uniform Search':<15} {int(theoretical_total_us):<12} {actual_us:<10} {actual_us-int(theoretical_total_us):<12} {'+' if actual_us <= theoretical_total_us else '-'}")
        print("-"*80)


def efficiency_comparison_plot():
    """
    Plot efficiency comparison between methods
    """
    a, b = 0.5, 1.0
    precisions = [1e-1, 5e-2, 1e-2, 5e-3, 1e-3, 5e-4, 1e-4]
    
    gs_evals = []
    us_evals = []
    
    for eps in precisions:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        gs_evals.append(gs_ev)
        
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        us_evals.append(us_ev)
    
    plt.figure(figsize=(10, 6))
    plt.semilogx(precisions, gs_evals, 'b-o', label='Golden Section', markersize=6)
    plt.semilogx(precisions, us_evals, 'r-s', label='Uniform Search', markersize=6)
    plt.xlabel('Required Precision (ε)')
    plt.ylabel('Number of Function Evaluations')
    plt.title('Efficiency Comparison: Function Evaluations vs Precision')
    plt.grid(True, alpha=0.3)
    plt.gca().invert_xaxis()  # Smaller epsilon means higher precision
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('plots/efficiency_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def main():
    """
    Main function to run all visualizations
    """
    print("Generating plots for optimization methods analysis...")
    
    # Create plots directory if it doesn't exist
    import os
    if not os.path.exists('plots'):
        os.makedirs('plots')
    
    # 1. Plot the target function to show unimodality
    print("Plotting target function...")
    plot_target_function()
    
    # 2. Compare methods at different precisions
    print("Comparing methods at different precisions...")
    compare_methods_at_different_precisions()
    
    # 3. Plot convergence history
    print("Plotting convergence history...")
    plot_convergence_history()
    
    # 4. Compare theoretical vs actual evaluations
    print("Comparing theoretical vs actual evaluations...")
    theoretical_vs_actual_evaluations()
    
    # 5. Efficiency comparison plot
    print("Creating efficiency comparison plot...")
    efficiency_comparison_plot()
    
    print("\nAll plots have been saved to the 'plots' directory.")


if __name__ == "__main__":
    main()