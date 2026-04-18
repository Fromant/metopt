import pandas as pd
import numpy as np

def f(x1, x2):
    return x1**2 + 2*x2**2 + np.exp(x1**2 + x2**2)

f_star = 1.0

df = pd.read_csv('cmake-build-debug/all_trajectories.csv')

method_full_names = {
    'GD': 'Градиентный спуск',
    'HJ': 'Хук-Дживс',
    'BFGS': 'БФГШ'
}

eps = 0.01
print(f"Отношение сходимости: (f(x_k+1) - f(x_*)) / (f(x_k) - f(x_*))")
print(f"   EPS = {eps}, f(x_*) = {f_star}\n")
print("=" * 80)

for base_method in ['GD', 'HJ', 'BFGS']:
    method_key = f"{base_method}_tol_{eps}"
    
    if method_key not in df['method'].values:
        print(f"\nМетод {method_key} не найден в данных")
        continue
    
    print(f"\n {method_full_names[base_method]} ({base_method})")
    print("-" * 80)
    print(f"{'Итерация':<10} {'f(x_k)':<15} {'f(x_k+1)':<15} {'Отношение':<15}")
    print("-" * 80)
    
    method_data = df[df['method'] == method_key].sort_values('iteration').reset_index(drop=True)
    
    prev_f = None
    prev_iter = -1
    global_iter = 0
    
    for idx, row in method_data.iterrows():
        current_f = row['f_value']
        current_iter = row['iteration']
        
        if prev_f is not None:
            denom = prev_f - f_star
            num = current_f - f_star
            
            if abs(denom) > 1e-15:
                ratio = num / denom
                print(f"{global_iter:<10} {prev_f:<15.8f} {current_f:<15.8f} {ratio:<15.8f}")
            else:
                print(f"{global_iter:<10} {prev_f:<15.8f} {current_f:<15.8f} {'-':<15}")
        
        prev_f = current_f
        prev_iter = current_iter
        global_iter += 1
    
    print("-" * 80)