import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
import math


def target_function(x):
    """
    The target function: f(x) = x^2 - 2x - 2cos(x)
    """
    return x**2 - 2*x - 2*np.cos(x)

def theoretical_golden_section_evals(initial_width, epsilon):
    """
    Теоретическое количество вычислений для метода золотого сечения.
    Формула: N = 3 + ceil( ln(L0/ε) / ln(φ) )
    """
    phi = (1 + np.sqrt(5)) / 2
    if initial_width <= epsilon:
        return 3
    k = np.ceil(np.log(initial_width / epsilon) / np.log(phi))
    return int(3 + k)


def theoretical_uniform_search_evals(initial_width, epsilon, points_per_iter=5):
    """
    Теоретическое количество вычислений для метода равномерного поиска (худший случай).
    Формула: N = m * ceil( ln(L0/ε) / ln((m-1)/2) ) + 1
    Для m=5: d = 2, N = 5 * ceil( ln(L0/ε) / ln(2) ) + 1
    """
    m = points_per_iter
    if initial_width <= epsilon:
        return m + 1
    d = (m - 1) / 2.0
    if d <= 1:
        d = 1.01 
    k = np.ceil(np.log(initial_width / epsilon) / np.log(d))
    return int(m * k + 1)


def theoretical_uniform_search_expected_evals(initial_width, epsilon, points_per_iter=5):
    """
    Ожидаемое (среднее) количество вычислений для равномерного поиска.
    Использует эффективный коэффициент сжатия d_eff ≈ 2.64 для m=5.
    """
    m = points_per_iter
    if initial_width <= epsilon:
        return m + 1
    d_eff = ((m-1)**(2/m)) * (((m-1)/2)**((m-2)/m))
    k = np.ceil(np.log(initial_width / epsilon) / np.log(d_eff))
    return int(m * k + 1)


def golden_section_search(func, a, b, epsilon, return_history=False):
    """
    Golden section search implementation
    """
    phi = (1 + np.sqrt(5)) / 2  
    resphi = 2 - phi  
    
    x1 = a + resphi * (b - a)
    x2 = b - resphi * (b - a)
    f1 = func(x1)
    f2 = func(x2)
    
    eval_count = 2  
    
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


def theoretical_formulas_analysis():
    """
    Display and verify the theoretical formulas for number of function evaluations
    """
    print("THEORETICAL FORMULAS ANALYSIS")
    print("="*60)
    
    a, b = 0.5, 1.0  # Interval width = 0.5
    precisions = [0.1, 0.01, 0.001]
    
    print("\nGOLDEN SECTION SEARCH:")
    print("Formula: n >= ln(|b0 - a0| / e) / ln(phi) + 2")
    print("where phi = (1+sqrt(5))/2 ~= 1.618")
    print(f"Initial interval width: |b0 - a0| = {b-a}")
    
    golden_ratio = (1 + np.sqrt(5)) / 2
    print(f"Golden ratio = {golden_ratio:.6f}")
    
    print(f"\n{'e':<10} {'Theoretical':<12} {'Actual':<10} {'Diff':<8} {'Match':<8}")
    print("-" * 50)
    
    for eps in precisions:
        theoretical = math.ceil(math.log((b - a) / eps) / math.log(golden_ratio)) + 2
        _, _, actual = golden_section_search(target_function, a, b, eps)
        diff = actual - theoretical
        match = "Y" if actual <= theoretical + 2 else "N"  # Allow small tolerance
        print(f"{eps:<10.4f} {theoretical:<12} {actual:<10} {diff:<8} {match:<8}")
    
    print("\nUNIFORM SEARCH:")
    print("For k points per iteration, after n iterations:")
    print("Interval width ~= |b0 - a0| * (2/(k-1))^n")
    print("So: n >= ln(e/|b0 - a0|) / ln(2/(k-1))")
    print("Total evaluations ~= n * k")
    print(f"Using k = 5 points per iteration")
    
    print(f"\n{'e':<10} {'Theoretical':<12} {'Actual':<10} {'Diff':<8} {'Match':<8}")
    print("-" * 50)
    
    for eps in precisions:
        if 2/(5-1) < 1:  # 0.5 < 1, so this works
            theoretical_iter = math.ceil(math.log(eps/(b-a)) / math.log(2/(5-1)))
        else:
            theoretical_iter = 1  # Fallback
        theoretical_total = theoretical_iter * 5
        
        _, _, actual = uniform_search(target_function, a, b, eps, points_per_iter=5)
        diff = actual - theoretical_total
        match = "Y" if abs(actual - theoretical_total) <= 5 else "N"  # Allow larger tolerance
        print(f"{eps:<10.4f} {theoretical_total:<12} {actual:<10} {diff:<8} {match:<8}")


def precision_accuracy_results():
    """
    Show results with precision: tenth, hundredth, thousandth
    """
    print("\n\nPRECISION ACCURACY RESULTS")
    print("="*80)
    
    a, b = 0.5, 1.0
    precisions = [0.1, 0.01, 0.001]
    precision_names = ["Tenth (0.1)", "Hundredth (0.01)", "Thousandth (0.001)"]
    
    print(f"{'Precision':<15} {'Method':<15} {'Min X':<12} {'Min Value':<15} {'Evals':<8} {'Interval Width':<15}")
    print("-" * 80)
    
    for i, eps in enumerate(precisions):
        # Golden Section
        gs_min_x, gs_min_val, gs_evals = golden_section_search(target_function, a, b, eps)
        _, _, _, gs_history = golden_section_search(target_function, a, b, eps, return_history=True)
        final_interval_gs = gs_history[-1][3]  # Last interval width
        
        print(f"{precision_names[i]:<15} {'Golden Section':<15} {gs_min_x:<12.6f} {gs_min_val:<15.6f} {gs_evals:<8} {final_interval_gs:<15.6f}")
        
        # Uniform Search
        us_min_x, us_min_val, us_evals = uniform_search(target_function, a, b, eps)
        _, _, _, us_history = uniform_search(target_function, a, b, eps, return_history=True)
        final_interval_us = us_history[-1][3]  # Last interval width
        
        print(f"{'':<15} {'Uniform Search':<15} {us_min_x:<12.6f} {us_min_val:<15.6f} {us_evals:<8} {final_interval_us:<15.6f}")
        print("-" * 80)


def method_comparison_visualization():
    """
    Create visualizations comparing both methods
    """
    a, b = 0.5, 1.0
    
    # Plot 1: Function with minimum marked
    x = np.linspace(0.4, 1.1, 1000)
    y = target_function(x)
    
    plt.figure(figsize=(15, 10))
    
    # Subplot 1: Function visualization
    plt.subplot(2, 3, 1)
    plt.plot(x, y, 'b-', linewidth=2, label=r'$f(x) = x^2 - 2x - 2\cos(x)$')
    result = minimize_scalar(target_function, bounds=(0.5, 1.0), method='bounded')
    plt.plot(result.x, result.fun, 'ro', markersize=10, label=f'True Min: x={result.x:.4f}')
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.title('Target Function: Unimodal Property')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Subplot 2: Convergence for ε = 0.001
    epsilon = 0.001
    _, _, _, gs_history = golden_section_search(target_function, a, b, epsilon, return_history=True)
    _, _, _, us_history = uniform_search(target_function, a, b, epsilon, return_history=True)
    
    gs_intervals = [h[3] for h in gs_history]
    us_intervals = [h[3] for h in us_history]
    gs_evals = [h[4] for h in gs_history]
    us_evals = [h[4] for h in us_history]
    
    plt.subplot(2, 3, 2)
    plt.semilogy(gs_evals, gs_intervals, 'b-o', label='Golden Section', markersize=4)
    plt.semilogy(us_evals, us_intervals, 'r-s', label='Uniform Search', markersize=4)
    plt.xlabel('Function Evaluations')
    plt.ylabel('Interval Width (Log Scale)')
    plt.title('Convergence Comparison (ε = 0.001)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Subplot 3: Precision vs Evaluations
    precisions = [1e-1, 5e-2, 1e-2, 5e-3, 1e-3, 5e-4, 1e-4]
    gs_evals_list = []
    us_evals_list = []
    
    for eps in precisions:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        gs_evals_list.append(gs_ev)
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        us_evals_list.append(us_ev)
    
    precisions = [1e-1, 5e-2, 1e-2, 5e-3, 1e-3, 5e-4, 1e-4]
    gs_evals_list = []
    us_evals_list = []
    gs_theory_list = []
    us_theory_list = []
    us_expected_list = []
    
    for eps in precisions:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        gs_evals_list.append(gs_ev)
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        us_evals_list.append(us_ev)
        
        gs_theory_list.append(theoretical_golden_section_evals(b-a, eps))
        us_theory_list.append(theoretical_uniform_search_evals(b-a, eps))
        us_expected_list.append(theoretical_uniform_search_expected_evals(b-a, eps))
    
    plt.subplot(2, 3, 3)
    plt.loglog(precisions, gs_evals_list, 'b-o', label='Golden Section (fact)', markersize=6)
    plt.loglog(precisions, us_evals_list, 'r-s', label='Uniform Search (fact)', markersize=6)
    plt.loglog(precisions, gs_theory_list, 'b--', label='Golden Section (theory)', linewidth=1.5, alpha=0.7)
    plt.loglog(precisions, us_theory_list, 'r--', label='Uniform Search (theory, worst-case)', linewidth=1.5, alpha=0.7)
    plt.loglog(precisions, us_expected_list, 'r:', label='Uniform Search (expected, d_eff≈2.64)', linewidth=1.5, alpha=0.7)
    
    plt.xlabel('Required Precision (ε)')
    plt.ylabel('Function Evaluations (Log Scale)')
    plt.title('Evaluations vs Precision')
    plt.grid(True, alpha=0.3, which='both')
    plt.legend(fontsize=8)
    plt.gca().invert_xaxis()
    
    # Subplot 4: Efficiency ratio
    efficiency_ratios = [gs/us for gs, us in zip(gs_evals_list, us_evals_list)]
    plt.subplot(2, 3, 4)
    plt.semilogx(precisions, efficiency_ratios, 'g-o', markersize=6)
    plt.axhline(y=1, color='k', linestyle='--', alpha=0.7, label='Equal Performance')
    plt.xlabel('Required Precision (ε)')
    plt.ylabel('GS Evals / US Evals')
    plt.title('Relative Efficiency')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.gca().invert_xaxis()
    
    # Subplot 5: Method behavior at different precisions
    precisions_detail = [0.1, 0.01, 0.001]
    prec_labels = ['0.1', '0.01', '0.001']
    
    gs_counts = []
    us_counts = []
    for eps in precisions_detail:
        _, _, gs_ev = golden_section_search(target_function, a, b, eps)
        _, _, us_ev = uniform_search(target_function, a, b, eps)
        gs_counts.append(gs_ev)
        us_counts.append(us_ev)
    
    x_pos = np.arange(len(precisions_detail))
    width = 0.35
    
    plt.subplot(2, 3, 5)
    plt.bar(x_pos - width/2, gs_counts, width, label='Golden Section', alpha=0.8)
    plt.bar(x_pos + width/2, us_counts, width, label='Uniform Search', alpha=0.8)
    plt.xlabel('Precision Level')
    plt.ylabel('Function Evaluations')
    plt.title('Evaluations at Key Precisions')
    plt.xticks(x_pos, prec_labels)
    plt.legend()
    
    # Subplot 6: Error convergence
    true_min_x = minimize_scalar(target_function, bounds=(0.5, 1.0), method='bounded').x
    _, _, _, gs_hist = golden_section_search(target_function, a, b, 0.001, return_history=True)
    _, _, _, us_hist = uniform_search(target_function, a, b, 0.001, return_history=True)
    
    gs_errors = [abs(h[2] - true_min_x) for h in gs_hist]
    us_errors = [abs(h[2] - true_min_x) for h in us_hist]
    gs_evals_err = [h[4] for h in gs_hist]
    us_evals_err = [h[4] for h in us_hist]
    
    plt.subplot(2, 3, 6)
    plt.semilogy(gs_evals_err, gs_errors, 'b-o', label='Golden Section', markersize=4)
    plt.semilogy(us_evals_err, us_errors, 'r-s', label='Uniform Search', markersize=4)
    plt.xlabel('Function Evaluations')
    plt.ylabel('|Estimate - True Min| (Log Scale)')
    plt.title('Error Convergence')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('plots/comprehensive_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def main():
    """
    Main function to run the complete analysis
    """
    print("FINAL ANALYSIS: Golden Section vs Uniform Search Methods")
    print("="*80)
    
    # Create plots directory if it doesn't exist
    import os
    if not os.path.exists('plots'):
        os.makedirs('plots')
    
    # Run all analyses
    theoretical_formulas_analysis()
    precision_accuracy_results()
    method_comparison_visualization()


if __name__ == "__main__":
    main()