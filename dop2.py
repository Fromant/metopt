import pandas as pd
import numpy as np

def f(x1, x2):
    return x1**2 + 2*x2**2 + np.exp(x1**2 + x2**2)

def grad_f(x1, x2):
    df_dx1 = 2*x1 + 2*x1*np.exp(x1**2 + x2**2)
    df_dx2 = 4*x2 + 2*x2*np.exp(x1**2 + x2**2)
    return np.array([df_dx1, df_dx2])

def grad_norm_squared(x1, x2):
    grad = grad_f(x1, x2)
    return np.dot(grad, grad)

df = pd.read_csv('cmake-build-debug/all_trajectories.csv')

eps = 0.01
method_key = f"GD_tol_{eps}"

print(f"📊 Отношение для градиентного спуска: ||grad(f(x_k))||² / (f(x_k) - f(x_k+1))")
print(f"   EPS = {eps}\n")
print("=" * 90)

if method_key not in df['method'].values:
    print(f"⚠️  Метод {method_key} не найден в данных")
    exit(1)

print(f"{'Итерация':<10} {'x1':<12} {'x2':<12} {'||grad||^2':<15} {'f(x_k)-f(x_k+1)':<18} {'Отношение':<15}")
print("=" * 90)

method_data = df[df['method'] == method_key].sort_values('iteration').reset_index(drop=True)

prev_f = None
prev_x1 = None
prev_x2 = None
prev_iter = -1
global_iter = 0

for idx, row in method_data.iterrows():
    current_f = row['f_value']
    current_x1 = row['x1']
    current_x2 = row['x2']
    current_iter = row['iteration']
    
    if prev_f is not None and prev_x1 is not None:
        grad_norm_sq = grad_norm_squared(prev_x1, prev_x2)
        f_diff = prev_f - current_f
        
        if abs(f_diff) > 1e-15:
            ratio = grad_norm_sq / f_diff
            print(f"{global_iter:<10} {prev_x1:<12.6f} {prev_x2:<12.6f} {grad_norm_sq:<15.8f} {f_diff:<18.8f} {ratio:<15.8f}")
        else:
            print(f"{global_iter:<10} {prev_x1:<12.6f} {prev_x2:<12.6f} {grad_norm_sq:<15.8f} {'~0':<18} {'-':<15}")
    
    prev_f = current_f
    prev_x1 = current_x1
    prev_x2 = current_x2
    prev_iter = current_iter
    global_iter += 1

print("=" * 90)