import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')

plt.style.use('default')

save_path = 'plots/'

def f(x1, x2):
    return x1**2 + 2*x2**2 + np.exp(x1**2 + x2**2)

f_star = 1.0
x_star = np.array([0.0, 0.0])

df = pd.read_csv('cmake-build-debug/all_trajectories.csv')

method_base_colors = {
    'GD': '#e74c3c',
    'HJ': '#3498db',
    'BFGS': '#2ecc71'
}

method_full_names = {
    'GD': 'Градиентный спуск',
    'HJ': 'Хук-Дживс',
    'BFGS': 'БФГШ'
}

epsilons = [0.1, 0.01, 0.001]

def split_trajectories(method_df):
    trajectories = []
    current = []
    prev_iter = -1
    
    for _, row in method_df.sort_values('iteration').iterrows():
        if row['iteration'] == 0 and prev_iter >= 0:
            trajectories.append(current)
            current = []
        current.append((row['x1'], row['x2'], row['f_value'], row['iteration']))
        prev_iter = row['iteration']
    if current:
        trajectories.append(current)
    return trajectories

print("📊 Генерация графиков нормы ||x - x*|| от итераций...\n")

for eps in epsilons:
    print(f"🔹 EPS = {eps}")
    
    df_eps = df[df['method'].str.contains(f'tol_{eps}')].copy()
    methods = ['GD', 'HJ', 'BFGS']
    
    fig_norm, ax_norm = plt.subplots(figsize=(13, 9))
    
    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            print(f"  ⚠️  Метод {method_key} не найден, пропускаем...")
            continue
            
        method_data = df_eps[df_eps['method'] == method_key].sort_values('iteration')
        trajectories = split_trajectories(method_data)
        
        all_iters = []
        all_norms = []
        global_iter = 0
        
        for traj in trajectories:
            for i, point in enumerate(traj):
                x1, x2 = point[0], point[1]
                norm = np.sqrt((x1 - x_star[0])**2 + (x2 - x_star[1])**2)
                all_norms.append(max(norm, 1e-10))
                all_iters.append(global_iter)
                global_iter += 1
        
        color = method_base_colors[base_method]
        label = method_full_names[base_method]
        
        print(f"  ✓ {label}: {len(all_iters)} итераций, финальная норма = {all_norms[-1]:.2e}")
        
        ax_norm.semilogy(all_iters, all_norms, marker='o', linewidth=2.5, 
                        color=color, label=label, alpha=0.9, markersize=5)
    
    ax_norm.set_xlabel('Итерация', fontsize=13)
    ax_norm.set_ylabel('Норма ||x - x*||', fontsize=13)
    ax_norm.set_title(f'Сходимость по расстоянию до оптимума (EPS = {eps})', fontsize=15)
    ax_norm.legend(loc='upper right', fontsize=11, framealpha=0.95)
    ax_norm.grid(True, alpha=0.3, linestyle='--', which='both')
    ax_norm.set_axisbelow(True)
    
    ax_norm.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{y:.0e}'))
    
    plt.savefig(save_path+f'norm_convergence_eps_{str(eps).replace(".", "_")}.png', 
                dpi=300, bbox_inches='tight')
    plt.show()
    
    print()

# ==================== Сводный график: финальная норма от точности ====================
print("📊 Создание сводного графика финальной нормы от точности...\n")

fig_summary, ax_summary = plt.subplots(figsize=(14, 9))

final_norms = {method: [] for method in ['GD', 'HJ', 'BFGS']}

for eps in epsilons:
    df_eps = df[df['method'].str.contains(f'tol_{eps}')].copy()
    
    for base_method in ['GD', 'HJ', 'BFGS']:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            final_norms[base_method].append(None)
            continue
        
        method_data = df_eps[df_eps['method'] == method_key].sort_values('iteration')
        trajectories = split_trajectories(method_data)
        
        # Берём последнюю точку последней траектории
        if trajectories:
            last_traj = trajectories[-1]
            last_point = last_traj[-1]
            x1, x2 = last_point[0], last_point[1]
            norm = np.sqrt((x1 - x_star[0])**2 + (x2 - x_star[1])**2)
            final_norms[base_method].append(norm)
        else:
            final_norms[base_method].append(None)

for method in ['GD', 'HJ', 'BFGS']:
    color = method_base_colors[method]
    label = method_full_names[method]
    
    valid_eps = [e for e, v in zip(epsilons, final_norms[method]) if v is not None]
    valid_norms = [v for v in final_norms[method] if v is not None]
    
    if valid_eps and valid_norms:
        ax_summary.plot(valid_eps, valid_norms, marker='o', linewidth=2.5, 
                       color=color, label=label, markersize=8, alpha=0.9)

ax_summary.set_xlabel('Точность (ε)', fontsize=13)
ax_summary.set_ylabel('Финальная норма ||x - x*||', fontsize=13)
ax_summary.set_title('Зависимость финальной нормы от точности', fontsize=15)
ax_summary.legend(loc='upper right', fontsize=11, framealpha=0.95)
ax_summary.grid(True, alpha=0.3, linestyle='--')
ax_summary.set_xscale('log')
ax_summary.set_yscale('log')

plt.savefig(save_path+'final_norm_vs_accuracy.png', dpi=300, bbox_inches='tight')
plt.show()

print("\n✅ Готово! Графики нормы сохранены.")
print(f"📁 Путь сохранения: {save_path}")
print("\n📄 Созданные файлы:")
for eps in epsilons:
    print(f"   • norm_convergence_eps_{str(eps).replace('.', '_')}.png")
print("   • final_norm_vs_accuracy.png")