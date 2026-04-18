import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

save_path = 'plots/'

def f(x1, x2):
    return x1**2 + 2*x2**2 + np.exp(x1**2 + x2**2)

def grad_f(x1, x2):
    df_dx1 = 2*x1 + 2*x1*np.exp(x1**2 + x2**2)
    df_dx2 = 4*x2 + 2*x2*np.exp(x1**2 + x2**2)
    return df_dx1, df_dx2

f_star = 1.0

df = pd.read_csv('cmake-build-debug/all_trajectories.csv')

# Цвета и названия методов
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

# Новые границы для графиков
x1_range = np.linspace(-1.2, 1.2, 100)
x2_range = np.linspace(-1.2, 1.2, 100)
X1, X2 = np.meshgrid(x1_range, x2_range)
Z = f(X1, X2)

Z_clipped = np.clip(Z, 1, 30)

def parse_method_name(name):
    parts = name.split('_tol_')
    base = parts[0]
    eps = float(parts[1]) if len(parts) > 1 else 0.1
    return base, eps

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

# Сбор данных для графика "итерации от точности"
all_convergence_data = {eps: {} for eps in epsilons}

for eps in epsilons:
    print(f"\n🔹 Генерация графиков для EPS = {eps}")

    df_eps = df[df['method'].str.contains(f'tol_{eps}')].copy()
    methods = ['GD', 'HJ', 'BFGS']

    # ==================== 3D График ====================
    fig_3d = plt.figure(figsize=(16, 12))
    ax = fig_3d.add_subplot(111, projection='3d')

    surf = ax.plot_surface(X1, X2, Z_clipped, cmap='viridis', alpha=0.4,
                           linewidth=0, antialiased=True, rstride=2, cstride=2,
                           vmin=1, vmax=30)

    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            continue

        method_data = df_eps[df_eps['method'] == method_key]
        trajectories = split_trajectories(method_data)

        color = method_base_colors[base_method]
        label = method_full_names[base_method]

        for traj in trajectories:
            x1_vals = [p[0] for p in traj]
            x2_vals = [p[1] for p in traj]
            z_vals = f(np.array(x1_vals), np.array(x2_vals))
            z_vals_clipped = np.clip(z_vals, 1, 30)

            ax.plot(x1_vals, x2_vals, z_vals_clipped, '-', color=color,
                    linewidth=3, label=label, alpha=0.95)

            ax.scatter(x1_vals, x2_vals, z_vals_clipped, s=50, c=color,
                       edgecolors='white', linewidths=1.5)

            ax.scatter(x1_vals[0], x2_vals[0], z_vals_clipped[0], s=150,
                       c=color, marker='*', edgecolors='black', linewidths=2.5, zorder=10)

    ax.scatter(0, 0, 1, s=300, c='gold', marker='*',
               edgecolors='black', linewidths=3, label='Минимум (0,0)', zorder=11)

    ax.set_xlabel('x1')
    ax.set_ylabel('x2')
    ax.set_zlabel('f(x)')
    ax.set_title(f'3D траектории оптимизации (EPS = {eps})')

    ax.legend(loc='upper left', fontsize=10, framealpha=0.95,
              bbox_to_anchor=(0.05, 0.95))

    # Камера параллельно оси x1 (elev=90, azim=0)
    ax.view_init(elev=90, azim=0)

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(1, 30)

    cbar = plt.colorbar(surf, ax=ax, shrink=0.6, aspect=15, label='f(x)')

    plt.savefig(save_path+f'surface_3d_eps_{str(eps).replace(".", "_")}.png',
                dpi=600, bbox_inches='tight')
    plt.show()

    # ==================== 2D Контурный график ====================
    fig_2d, ax = plt.subplots(figsize=(14, 10))

    contour = ax.contourf(X1, X2, Z_clipped, levels=60, cmap='viridis', alpha=0.7,
                          vmin=1, vmax=50)
    ax.contour(X1, X2, Z_clipped, levels=25, colors='white', linewidths=0.4, alpha=0.5)

    skip = 8
    Xg, Yg = np.meshgrid(x1_range[::skip], x2_range[::skip])
    U, V = grad_f(Xg, Yg)
    mag = np.sqrt(U**2 + V**2) + 1e-10
    ax.quiver(Xg, Yg, U/mag, V/mag, color='gray', alpha=0.5, scale=50, width=0.015)

    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            continue

        method_data = df_eps[df_eps['method'] == method_key]
        trajectories = split_trajectories(method_data)

        color = method_base_colors[base_method]
        label = method_full_names[base_method]

        for traj in trajectories:
            x1_vals = [p[0] for p in traj]
            x2_vals = [p[1] for p in traj]

            ax.plot(x1_vals, x2_vals, '-', color=color, linewidth=2.5,
                    label=label, alpha=0.9)

            ax.scatter(x1_vals, x2_vals, s=40, c=color, edgecolors='white',
                       linewidths=1.2, alpha=0.95)

            ax.scatter(x1_vals[0], x2_vals[0], s=130, c=color, marker='*',
                       edgecolors='black', linewidths=2, zorder=5)

    ax.scatter(0, 0, s=280, c='gold', marker='*', edgecolors='black',
               linewidths=2.5, label='Минимум (0, 0)', zorder=6)

    ax.set_xlabel('x1')
    ax.set_ylabel('x₂')
    ax.set_title(f'Траектории оптимизации (EPS = {eps})')
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)

    cbar = plt.colorbar(contour, ax=ax, label='f(x)', fraction=0.046, pad=0.04)

    plt.savefig(save_path+f'contour_eps_{str(eps).replace(".", "_")}.png', dpi=300, bbox_inches='tight')
    plt.show()

    # ==================== График сходимости ====================
    fig_conv, ax = plt.subplots(figsize=(13, 9))

    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            continue

        method_data = df_eps[df_eps['method'] == method_key].sort_values('iteration')

        errors = []
        iters = []
        global_iter = 0
        prev_iter = -1

        for _, row in method_data.iterrows():
            if row['iteration'] == 0 and prev_iter >= 0:
                global_iter = 0
            err = abs(row['f_value'] - f_star)
            errors.append(max(err, 1e-10))
            iters.append(global_iter)
            global_iter += 1
            prev_iter = row['iteration']

        color = method_base_colors[base_method]
        label = method_full_names[base_method]

        ax.semilogy(iters, errors, marker='o', linewidth=2.5,
                    color=color, label=label, alpha=0.9, markersize=5)

        # Сохраняем данные для сводного графика
        all_convergence_data[eps][base_method] = {
            'iters': iters,
            'errors': errors,
            'color': color,
            'label': label
        }

    ax.set_xlabel('Итерация')
    ax.set_ylabel('Абсолютная погрешность |f(x) - f*|')
    ax.set_title(f'Сходимость методов (EPS = {eps})')
    ax.legend(loc='upper right', fontsize=10, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--', which='both')
    ax.set_axisbelow(True)

    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{y:.0e}'))

    plt.savefig(save_path+f'convergence_eps_{str(eps).replace(".", "_")}.png', dpi=300, bbox_inches='tight')
    plt.show()

    # ==================== Анимированный 2D график (полный вид) ====================
    print(f"  🎬 Создание анимации для EPS = {eps} (полный вид)...")

    fig_anim, ax_anim = plt.subplots(figsize=(14, 10))

    contour_anim = ax_anim.contourf(X1, X2, Z_clipped, levels=60, cmap='viridis', alpha=0.7,
                                    vmin=1, vmax=50)
    ax_anim.contour(X1, X2, Z_clipped, levels=25, colors='white', linewidths=0.4, alpha=0.5)

    ax_anim.scatter(0, 0, s=280, c='gold', marker='*', edgecolors='black',
                    linewidths=2.5, label='Минимум (0, 0)', zorder=6)

    # Линии траекторий (будут обновляться)
    traj_lines = {}
    traj_points = {}
    traj_starts = {}

    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            continue

        color = method_base_colors[base_method]
        label = method_full_names[base_method]

        method_data = df_eps[df_eps['method'] == method_key]
        trajectories = split_trajectories(method_data)

        traj_lines[base_method] = []
        traj_points[base_method] = []
        traj_starts[base_method] = []

        for traj in trajectories:
            x1_vals = [p[0] for p in traj]
            x2_vals = [p[1] for p in traj]

            line, = ax_anim.plot([], [], '-', color=color, linewidth=2.5, alpha=0.9)
            points = ax_anim.scatter([], [], s=40, c=color, edgecolors='white',
                                     linewidths=1.2, alpha=0.95, zorder=5)
            start = ax_anim.scatter(x1_vals[0], x2_vals[0], s=130, c=color, marker='*',
                                    edgecolors='black', linewidths=2, zorder=5)

            traj_lines[base_method].append({'line': line, 'x1': x1_vals, 'x2': x2_vals})
            traj_points[base_method].append(points)
            traj_starts[base_method].append(start)

    ax_anim.set_xlabel('x1')
    ax_anim.set_ylabel('x₂')
    ax_anim.set_title(f'Анимация оптимизации (EPS = {eps})')
    ax_anim.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax_anim.grid(True, alpha=0.3, linestyle='--')
    ax_anim.set_aspect('equal')
    ax_anim.set_xlim(-1.2, 1.2)
    ax_anim.set_ylim(-1.2, 1.2)

    # Находим максимальное количество кадров
    max_frames = 0
    for base_method in methods:
        if base_method in traj_lines:
            for traj in traj_lines[base_method]:
                max_frames = max(max_frames, len(traj['x1']))

    iter_text = ax_anim.text(0.02, 0.98, '', transform=ax_anim.transAxes,
                             fontsize=12, verticalalignment='top',
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def animate(frame):
        for base_method in methods:
            if base_method not in traj_lines:
                continue

            for i, traj in enumerate(traj_lines[base_method]):
                if frame < len(traj['x1']):
                    traj['line'].set_data(traj['x1'][:frame+1], traj['x2'][:frame+1])
                    traj_points[base_method][i].set_offsets(
                        np.column_stack([traj['x1'][:frame+1], traj['x2'][:frame+1]])
                    )

        iter_text.set_text(f'Итерация: {frame}')
        return list(sum([[t['line'] for t in traj_lines.get(m, [])] for m in methods], [])) + \
            list(sum([list(traj_points.get(m, [])) for m in methods], [])) + [iter_text]

    anim = FuncAnimation(fig_anim, animate, frames=max_frames, interval=1000, blit=True)

    anim.save(save_path+f'animation_eps_{str(eps).replace(".", "_")}.gif',
              writer='pillow', fps=10, dpi=150)
    plt.close()

    print(f"  ✅ Анимация (полный вид) сохранена!")

    # ==================== Анимированный 2D график (зазумированный) ====================
    print(f"  🔍 Создание анимации для EPS = {eps} (зазумированный вид)...")

    # Границы для зумированной версии
    zoom_range = eps * 10
    x1_range_zoom = np.linspace(-zoom_range, zoom_range, 100)
    x2_range_zoom = np.linspace(-zoom_range, zoom_range, 100)
    X1_zoom, X2_zoom = np.meshgrid(x1_range_zoom, x2_range_zoom)
    Z_zoom = f(X1_zoom, X2_zoom)
    Z_zoom_clipped = np.clip(Z_zoom, 1, 30)

    fig_anim_zoom, ax_anim_zoom = plt.subplots(figsize=(14, 10))

    contour_anim_zoom = ax_anim_zoom.contourf(X1_zoom, X2_zoom, Z_zoom_clipped, levels=60,
                                              cmap='viridis', alpha=0.7, vmin=1, vmax=30)
    ax_anim_zoom.contour(X1_zoom, X2_zoom, Z_zoom_clipped, levels=25, colors='white',
                         linewidths=0.4, alpha=0.5)

    ax_anim_zoom.scatter(0, 0, s=280, c='gold', marker='*', edgecolors='black',
                         linewidths=2.5, label='Минимум (0, 0)', zorder=6)

    # Линии траекторий для зумированной версии
    traj_lines_zoom = {}
    traj_points_zoom = {}
    traj_starts_zoom = {}

    for base_method in methods:
        method_key = f"{base_method}_tol_{eps}"
        if method_key not in df_eps['method'].values:
            continue

        color = method_base_colors[base_method]
        label = method_full_names[base_method]

        method_data = df_eps[df_eps['method'] == method_key]
        trajectories = split_trajectories(method_data)

        traj_lines_zoom[base_method] = []
        traj_points_zoom[base_method] = []
        traj_starts_zoom[base_method] = []

        for traj in trajectories:
            x1_vals = [p[0] for p in traj]
            x2_vals = [p[1] for p in traj]

            line, = ax_anim_zoom.plot([], [], '-', color=color, linewidth=2.5, alpha=0.9)
            points = ax_anim_zoom.scatter([], [], s=40, c=color, edgecolors='white',
                                          linewidths=1.2, alpha=0.95, zorder=5)
            start = ax_anim_zoom.scatter(x1_vals[0], x2_vals[0], s=130, c=color, marker='*',
                                         edgecolors='black', linewidths=2, zorder=5)

            traj_lines_zoom[base_method].append({'line': line, 'x1': x1_vals, 'x2': x2_vals})
            traj_points_zoom[base_method].append(points)
            traj_starts_zoom[base_method].append(start)

    ax_anim_zoom.set_xlabel('x1')
    ax_anim_zoom.set_ylabel('x₂')
    ax_anim_zoom.set_title(f'Анимация оптимизации (EPS = {eps}, зум)')
    ax_anim_zoom.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax_anim_zoom.grid(True, alpha=0.3, linestyle='--')
    ax_anim_zoom.set_aspect('equal')
    ax_anim_zoom.set_xlim(-zoom_range, zoom_range)
    ax_anim_zoom.set_ylim(-zoom_range, zoom_range)

    # Находим максимальное количество кадров
    max_frames_zoom = 0
    for base_method in methods:
        if base_method in traj_lines_zoom:
            for traj in traj_lines_zoom[base_method]:
                max_frames_zoom = max(max_frames_zoom, len(traj['x1']))

    iter_text_zoom = ax_anim_zoom.text(0.02, 0.98, '', transform=ax_anim_zoom.transAxes,
                                       fontsize=12, verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    def animate_zoom(frame):
        for base_method in methods:
            if base_method not in traj_lines_zoom:
                continue

            for i, traj in enumerate(traj_lines_zoom[base_method]):
                if frame < len(traj['x1']):
                    traj['line'].set_data(traj['x1'][:frame+1], traj['x2'][:frame+1])
                    traj_points_zoom[base_method][i].set_offsets(
                        np.column_stack([traj['x1'][:frame+1], traj['x2'][:frame+1]])
                    )

        iter_text_zoom.set_text(f'Итерация: {frame}')
        return list(sum([[t['line'] for t in traj_lines_zoom.get(m, [])] for m in methods], [])) + \
            list(sum([list(traj_points_zoom.get(m, [])) for m in methods], [])) + [iter_text_zoom]

    anim_zoom = FuncAnimation(fig_anim_zoom, animate_zoom, frames=max_frames_zoom, interval=1000, blit=True)

    anim_zoom.save(save_path+f'animation_zoom_eps_{str(eps).replace(".", "_")}.gif',
                   writer='pillow', fps=10, dpi=150)
    plt.close()

    print(f"  ✅ Анимация (зазумированный вид) сохранена!")

# ==================== Сводный график: итерации от точности ====================
print("\n📊 Создание сводного графика зависимости итераций от точности...")

fig_summary, ax_summary = plt.subplots(figsize=(14, 9))

eps_for_plot = []
iters_for_plot = {method: [] for method in ['GD', 'HJ', 'BFGS']}

for eps in epsilons:
    eps_for_plot.append(eps)
    for method in ['GD', 'HJ', 'BFGS']:
        if method in all_convergence_data[eps]:
            iters_for_plot[method].append(len(all_convergence_data[eps][method]['iters']))
        else:
            iters_for_plot[method].append(None)

for method in ['GD', 'HJ', 'BFGS']:
    color = method_base_colors[method]
    label = method_full_names[method]

    # Фильтруем None значения
    valid_eps = [e for e, v in zip(epsilons, iters_for_plot[method]) if v is not None]
    valid_iters = [v for v in iters_for_plot[method] if v is not None]

    if valid_eps and valid_iters:
        ax_summary.plot(valid_eps, valid_iters, marker='o', linewidth=2.5,
                        color=color, label=label, markersize=8, alpha=0.9)

ax_summary.set_xlabel('Точность (ε)')
ax_summary.set_ylabel('Количество итераций')
ax_summary.set_title('Зависимость количества итераций от точности')
ax_summary.legend(loc='upper right', fontsize=11, framealpha=0.95)
ax_summary.grid(True, alpha=0.3, linestyle='--')
ax_summary.set_xscale('log')

plt.savefig(save_path+'iterations_vs_accuracy.png', dpi=300, bbox_inches='tight')
plt.show()

print("\n✅ Готово! Все графики и анимации сохранены.")
print(f"📁 Путь сохранения: {save_path}")