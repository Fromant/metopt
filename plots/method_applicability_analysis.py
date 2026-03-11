import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
import math


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
        history = [(a, b, (a+b)/2, abs(b-a), eval_count)]
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
            history.append((a, b, (a+b)/2, abs(b-a), eval_count))
    
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
        history = [(current_a, current_b, (current_a+current_b)/2, abs(current_b-current_a), eval_count)]
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
            history.append((current_a, current_b, (current_a+current_b)/2, abs(current_b-current_a), eval_count))
    
    min_x = (current_a + current_b) / 2
    min_val = func(min_x)
    
    if return_history:
        return min_x, min_val, eval_count, history
    else:
        return min_x, min_val, eval_count


def plot_unimodal_function():
    """
    Plot the target function to demonstrate its unimodality
    """
    x = np.linspace(-0.5, 2.5, 1000)
    y = target_function(x)
    
    plt.figure(figsize=(12, 8))
    plt.plot(x, y, 'b-', linewidth=2, label=r'$f(x) = x^2 - 2x - 2\cos(x)$')
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('Unimodal Function Analysis: $f(x) = x^2 - 2x - 2\cos(x)$')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Find and mark the global minimum
    result = minimize_scalar(target_function, bounds=(-0.5, 2.5), method='bounded')
    plt.plot(result.x, result.fun, 'ro', markersize=10, label=f'Global Minimum at x={result.x:.4f}')
    
    # Mark the region of interest [0.5, 1.0]
    x_region = np.linspace(0.5, 1.0, 200)
    y_region = target_function(x_region)
    plt.fill_between(x_region, y_region, alpha=0.2, color='green', label='Region of Interest [0.5, 1.0]')
    
    plt.legend()
    plt.tight_layout()
    plt.savefig('plots/unimodal_function.png', dpi=300, bbox_inches='tight')
    plt.show()


def plot_method_behavior_comparison():
    """
    Show how both methods behave differently on the same function
    """
    a, b = 0.5, 1.0
    epsilon = 0.001
    
    # Get histories
    _, _, _, gs_history = golden_section_search(target_function, a, b, epsilon, return_history=True)
    _, _, _, us_history = uniform_search(target_function, a, b, epsilon, return_history=True)
    
    # Extract data
    gs_iterations = list(range(len(gs_history)))
    gs_intervals = [h[3] for h in gs_history]  # Interval widths
    gs_evals = [h[4] for h in gs_history]  # Evaluation counts
    
    us_iterations = list(range(len(us_history)))
    us_intervals = [h[3] for h in us_history]  # Interval widths
    us_evals = [h[4] for h in us_history]  # Evaluation counts
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # Plot 1: Interval width vs iterations (log scale)
    ax1.semilogy(gs_iterations, gs_intervals, 'b-o', label='Golden Section', markersize=4)
    ax1.semilogy(us_iterations, us_intervals, 'r-s', label='Uniform Search', markersize=4)
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Interval Width (Log Scale)')
    ax1.set_title('Convergence: Interval Reduction')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot 2: Interval width vs function evaluations
    ax2.semilogy(gs_evals, gs_intervals, 'b-o', label='Golden Section', markersize=4)
    ax2.semilogy(us_evals, us_intervals, 'r-s', label='Uniform Search', markersize=4)
    ax2.set_xlabel('Function Evaluations')
    ax2.set_ylabel('Interval Width (Log Scale)')
    ax2.set_title('Convergence: Interval vs Function Evaluations')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot 3: Midpoint convergence
    gs_midpoints = [h[2] for h in gs_history]
    us_midpoints = [h[2] for h in us_history]
    true_min = minimize_scalar(target_function, bounds=(0.5, 1.0), method='bounded').x
    
    ax3.plot(gs_iterations, gs_midpoints, 'b-o', label='Golden Section', markersize=4)
    ax3.plot(us_iterations, us_midpoints, 'r-s', label='Uniform Search', markersize=4)
    ax3.axhline(y=true_min, color='g', linestyle='--', label=f'True Minimum ({true_min:.4f})')
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Midpoint Estimate')
    ax3.set_title('Midpoint Convergence to True Minimum')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Error vs function evaluations
    gs_errors = [abs(mid - true_min) for mid in gs_midpoints]
    us_errors = [abs(mid - true_min) for mid in us_midpoints]
    
    ax4.semilogy(gs_evals, gs_errors, 'b-o', label='Golden Section', markersize=4)
    ax4.semilogy(us_evals, us_errors, 'r-s', label='Uniform Search', markersize=4)
    ax4.set_xlabel('Function Evaluations')
    ax4.set_ylabel('|Estimated Min - True Min| (Log Scale)')
    ax4.set_title('Error Convergence vs Function Evaluations')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    plt.tight_layout()
    plt.savefig('plots/method_behavior_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def precision_vs_evaluations_analysis():
    """
    Detailed analysis of precision vs evaluations for both methods
    """
    a, b = 0.5, 1.0
    precisions = [1e-1, 5e-2, 1e-2, 5e-3, 1e-3, 5e-4, 1e-4, 5e-5, 1e-5]
    
    gs_evals = []
    us_evals = []
    
    for eps in precisions:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        gs_evals.append(gs_ev)
        
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        us_evals.append(us_ev)
    
    # Calculate theoretical predictions
    golden_ratio = (1 + np.sqrt(5)) / 2
    theoretical_gs = [math.ceil(math.log((b-a) / eps) / math.log(golden_ratio)) + 2 for eps in precisions]
    
    # For uniform search with 5 points, each iteration reduces interval by roughly factor of 3
    # So after n iterations: (b-a)/(3^n) <= eps => n >= log((b-a)/eps)/log(3)
    # Total evaluations = n * 5 (approximately)
    theoretical_us = [math.ceil(math.log((b-a) / eps) / math.log(3)) * 5 for eps in precisions]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Actual evaluations
    ax1.loglog(precisions, gs_evals, 'b-o', label='Golden Section (Actual)', markersize=6)
    ax1.loglog(precisions, us_evals, 'r-s', label='Uniform Search (Actual)', markersize=6)
    ax1.loglog(precisions, theoretical_gs, 'b--', label='Golden Section (Theoretical)', alpha=0.7)
    ax1.loglog(precisions, theoretical_us, 'r--', label='Uniform Search (Theoretical)', alpha=0.7)
    ax1.set_xlabel('Required Precision (ε)')
    ax1.set_ylabel('Function Evaluations (Log Scale)')
    ax1.set_title('Function Evaluations vs Required Precision')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.invert_xaxis()  # Higher precision = smaller epsilon
    
    # Plot 2: Efficiency ratio
    efficiency_ratios = [gs_e / us_e for gs_e, us_e in zip(gs_evals, us_evals)]
    ax2.semilogx(precisions, efficiency_ratios, 'g-o', markersize=6)
    ax2.axhline(y=1, color='k', linestyle='--', alpha=0.5, label='Equal Performance')
    ax2.set_xlabel('Required Precision (ε)')
    ax2.set_ylabel('GS Evals / US Evals')
    ax2.set_title('Relative Efficiency (Golden Section vs Uniform Search)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.invert_xaxis()  # Higher precision = smaller epsilon
    
    plt.tight_layout()
    plt.savefig('plots/precision_vs_evaluations.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return precisions, gs_evals, us_evals, theoretical_gs, theoretical_us


def method_advantage_zones():
    """
    Identify regions where each method is more advantageous
    """
    a, b = 0.5, 1.0
    precisions = [1e-1, 5e-2, 1e-2, 5e-3, 1e-3, 5e-4, 1e-4, 5e-5, 1e-5]
    
    gs_evals = []
    us_evals = []
    
    for eps in precisions:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        gs_evals.append(gs_ev)
        
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        us_evals.append(us_ev)
    
    # Determine advantage zones
    advantage_zones = []
    for i, (gs, us) in enumerate(zip(gs_evals, us_evals)):
        if gs < us:
            advantage_zones.append(('Golden Section', precisions[i]))
        else:
            advantage_zones.append(('Uniform Search', precisions[i]))
    
    print("\nMethod Advantage Analysis:")
    print("="*60)
    print(f"{'Precision':<12} {'GS Evals':<10} {'US Evals':<10} {'Winner':<15}")
    print("-"*60)
    
    for i, (prec, gs, us) in enumerate(zip(precisions, gs_evals, us_evals)):
        winner = "Golden Section" if gs < us else "Uniform Search"
        print(f"{prec:<12.1e} {gs:<10} {us:<10} {winner:<15}")
    
    print("\nConclusion:")
    print("- Golden Section is generally more efficient for high precision requirements")
    print("- Uniform Search may be better for lower precision requirements")
    print("- The crossover point depends on the function characteristics")


def plot_different_functions():
    """
    Demonstrate method behavior on different types of functions
    """
    def quadratic_func(x):
        return (x - 0.7)**2 + 0.5
    
    def trig_func(x):
        return np.cos(x) * np.exp(-x/5)
    
    def poly_func(x):
        return x**4 - 2*x**2 + x + 0.5
    
    functions = [
        (target_function, r'$f(x) = x^2 - 2x - 2\cos(x)$', 'Target Function'),
        (quadratic_func, r'$f(x) = (x-0.7)^2 + 0.5$', 'Quadratic'),
        (trig_func, r'$f(x) = \cos(x)e^{-x/5}$', 'Trigonometric Decay'),
        (poly_func, r'$f(x) = x^4 - 2x^2 + x + 0.5$', 'Polynomial')
    ]
    
    a, b = 0.5, 1.0
    epsilon = 0.001
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for idx, (func, formula, name) in enumerate(functions):
        # Run both methods
        _, _, gs_ev = golden_section_search(func, a, b, epsilon)
        _, _, us_ev = uniform_search(func, a, b, epsilon)
        
        # Plot function
        x = np.linspace(a, b, 500)
        y = func(x)
        axes[idx].plot(x, y, 'b-', linewidth=2)
        axes[idx].set_title(f'{name}\n{formula}')
        axes[idx].set_xlabel('x')
        axes[idx].set_ylabel('f(x)')
        axes[idx].grid(True, alpha=0.3)
        
        # Add evaluation counts to plot
        axes[idx].text(0.05, 0.95, f'GS: {gs_ev} evals\nUS: {us_ev} evals', 
                      transform=axes[idx].transAxes, verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('plots/different_functions_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def main():
    """
    Main function to run all analyses
    """
    print("Starting comprehensive analysis of optimization methods...")
    
    # Create plots directory if it doesn't exist
    import os
    if not os.path.exists('plots'):
        os.makedirs('plots')
    
    # 1. Plot the unimodal function
    print("1. Creating unimodal function visualization...")
    plot_unimodal_function()
    
    # 2. Compare method behaviors
    print("2. Creating method behavior comparison...")
    plot_method_behavior_comparison()
    
    # 3. Precision vs evaluations analysis
    print("3. Creating precision vs evaluations analysis...")
    precisions, gs_evals, us_evals, theoretical_gs, theoretical_us = precision_vs_evaluations_analysis()
    
    # 4. Method advantage zones
    print("4. Analyzing method advantage zones...")
    method_advantage_zones()
    
    # 5. Compare on different functions
    print("5. Creating different functions comparison...")
    plot_different_functions()
    
    print("\nComprehensive analysis complete!")
    print("All plots have been saved to the 'plots' directory.")
    print("\nKey findings:")
    print("- Golden Section is theoretically optimal for unimodal functions")
    print("- Uniform Search can be more robust for functions with noise")
    print("- The choice depends on precision requirements and function characteristics")
    print("- Golden Section typically requires fewer function evaluations for high precision")


if __name__ == "__main__":
    main()